// #include "src/memllib/interface/InterfaceBase.hpp"
#include "src/memllib/interface/MIDIInOut.hpp"
#include "src/memllib/hardware/memlnaut/display.hpp"
#include "src/memllib/audio/AudioAppBase.hpp"
#include "src/memllib/audio/AudioDriver.hpp"
#include "src/memllib/hardware/memlnaut/MEMLNaut.hpp"
#include <memory>
#include "src/memllib/interface/InterfaceRL.hpp"
//#include "src/memllib/synth/maxiPAF.hpp"
#include "hardware/structs/bus_ctrl.h"
#include "src/memllib/utils/sharedMem.hpp"
#include <VFS.h>
#include <LittleFS.h>
#include "src/memllib/interface/UARTInput.hpp"
#include <algorithm>
#include "KassiaAudioApp.hpp"


#define USE_JOYSTICK    0

display APP_SRAM scr;

bool core1_disable_systick = true;
bool core1_separate_stack = true;


uint32_t get_rosc_entropy_seed(int bits) {
    uint32_t seed = 0;
    for (int i = 0; i < bits; ++i) {
        // Wait for a bit of time to allow jitter to accumulate
        busy_wait_us_32(5);
        // Pull LSB from ROSC rand output
        seed <<= 1;
        seed |= (rosc_hw->randombit & 1);
    }
    return seed;
}


// Global objects
std::shared_ptr<InterfaceRL> APP_SRAM RLInterface;

std::shared_ptr<MIDIInOut> midi_interf;
std::shared_ptr<UARTInput> pio_uart;

std::shared_ptr<KassiaAudioApp> __scratch_y("audio") audio_app;

// Inter-core communication
volatile bool APP_SRAM core_0_ready = false;
volatile bool APP_SRAM core_1_ready = false;
volatile bool APP_SRAM serial_ready = false;
volatile bool APP_SRAM interface_ready = false;

#if USE_JOYSTICK
constexpr size_t kN_InputParams = 3;
#else
// KASSIA: set number of serial params
const std::vector<size_t> kSensorIndexes = {2, 3, 4, 5};
constexpr size_t kN_InputParams = 4;
#endif


struct repeating_timer APP_SRAM timerDisplay;
inline bool __not_in_flash_func(displayUpdate)(__unused struct repeating_timer *t) {
    scr.update();
    return true;
}

void setup()
{
    Serial.begin(115200);
    //while (!Serial) {}
    Serial.println("Serial initialised.");
    WRITE_VOLATILE(serial_ready, true);

    LittleFS.begin();
    VFS.root(LittleFS);

    scr.setup();
    bus_ctrl_hw->priority = BUSCTRL_BUS_PRIORITY_DMA_W_BITS |
        BUSCTRL_BUS_PRIORITY_DMA_R_BITS | BUSCTRL_BUS_PRIORITY_PROC1_BITS;

        uint32_t seed = get_rosc_entropy_seed(32);
    srand(seed);


    // FILE *fp = fopen("/thisfilelivesonflash.txt", "r");
    // char line[128];
    // while (fgets(line, sizeof(line), fp)) {
    //     Serial.printf("%s", line);
    // }

    // fclose(fp);

    // Setup board
    MEMLNaut::Initialize();

#if !USE_JOYSTICK
    pio_uart = std::make_shared<UARTInput>(
        kSensorIndexes,
        Pins::SENSOR_RX,
        Pins::SENSOR_TX,
        115200
    );
#endif

    // Set up interface
    auto temp_interface = std::make_shared<InterfaceRL>();
    temp_interface->setup(kN_InputParams, KassiaAudioApp::kN_Params);
    MEMORY_BARRIER();
    RLInterface = temp_interface;
    MEMORY_BARRIER();
    // Setup interface with memory barrier protection
    WRITE_VOLATILE(interface_ready, true);
    // Bind interface after ensuring it's fully initialized
    RLInterface->bind_RL_interface(scr); // Updated call
    Serial.println("Bound RL interface to MEMLNaut.");
    // Other UI init
    MEMLNaut::Instance()->setRVGain1Callback([](float value) { // Does not need 'this' or 'scr_ref'
        AudioDriver::setDACVolume(value);
    });


    midi_interf = std::make_shared<MIDIInOut>();
    midi_interf->Setup(4);
    midi_interf->SetMIDISendChannel(1);
    Serial.println("MIDI setup complete.");
    if (midi_interf) {
        midi_interf->SetCCCallback([RLInterface] (uint8_t cc_number, uint8_t cc_value) { // Capture RLInterface
            Serial.printf("MIDI CC %d: %d\n", cc_number, cc_value);
            switch(cc_number) {
                case 1:
                {
                    RLInterface->trigger_like(); // Updated call
                    break;
                }
                case 2:
                {
                    RLInterface->trigger_dislike(); // Updated call
                    break;
                }
                case 3:
                {
                    RLInterface->trigger_randomiseRL(); // Updated call
                    break;
                }
                case 4:
                {
                    RLInterface->forgetMemory();
                    Serial.println("Memory cleared.");
                    scr.post("What am I? A teapot?");
                    break;
                }
            };
        });
    }

    // PIO UART setup and callback
    if (pio_uart) {
        // pio_uart->SetCallback([RLInterface] (const std::vector<float>& values) {
        //     for (size_t i = 0; i < values.size() && i < kN_InputParams; ++i) {
        //         Serial.print(values[i]);
        //         Serial.print(" ");
        //     }
        //     Serial.println();
        // });
        pio_uart->SetCallback([RLInterface] (size_t sensor_index, float value) {
            // Find param index based on kSensorIndexes
            auto it = std::find(kSensorIndexes.begin(), kSensorIndexes.end(), sensor_index);
            if (it != kSensorIndexes.end()) {
                size_t param_index = std::distance(kSensorIndexes.begin(), it);
                Serial.printf("Sensor %zu: %f\n", sensor_index, value);
                RLInterface->setState(param_index, value);
            } else {
                Serial.printf("Invalid sensor index: %zu\n", sensor_index);
            }
        });
    }

    WRITE_VOLATILE(core_0_ready, true);
    while (!READ_VOLATILE(core_1_ready)) {
        MEMORY_BARRIER();
        delay(1);
    }

    scr.post("MEMLNaut: let's go!");
    add_repeating_timer_ms(-39, displayUpdate, NULL, &timerDisplay);

    Serial.println("Finished initialising core 0.");
}

void loop()
{
    static uint32_t last_1ms = 0;
    static uint32_t last_10ms = 0;
    static uint32_t last_1s = 0;
    static bool led_pulse = false;
    uint32_t current_time = millis();

    // 1ms tasks
    if (current_time - last_1ms >= 1) {
        // PIO UART reads
        if (pio_uart) {
            pio_uart->Poll();
        }
        // MEMLNaut pots
        MEMLNaut::Instance()->loop();
        // MIDI reads
        if (midi_interf) {
            midi_interf->Poll();
        }
        last_1ms = current_time;
    }

    // 10ms tasks
    if (current_time - last_10ms >= 10) {
        // Turn off LED if it was pulsed
        if (led_pulse) {
            digitalWrite(Pins::LED, LOW);
            led_pulse = false;
        }
        last_10ms = current_time;
    }

    // 1 second tasks
    if (current_time - last_1s >= 1000) {
        // Pulse LED on
        digitalWrite(Pins::LED, HIGH);
        led_pulse = true;
        Serial.println(".");
        last_1s = current_time;
    }
}

void setup1()
{
    while (!READ_VOLATILE(serial_ready)) {
        MEMORY_BARRIER();
        delay(1);
    }

    while (!READ_VOLATILE(interface_ready)) {
        MEMORY_BARRIER();
        delay(1);
    }


    // Create audio app with memory barrier protection
    {
        auto temp_audio_app = std::make_shared<KassiaAudioApp>();
        std::shared_ptr<InterfaceBase> selectedInterface;
        selectedInterface = std::dynamic_pointer_cast<InterfaceBase>(RLInterface);

        temp_audio_app->Setup(AudioDriver::GetSampleRate(), selectedInterface);
        MEMORY_BARRIER();
        audio_app = temp_audio_app;
        MEMORY_BARRIER();
    }

    // Start audio driver
    AudioDriver::Setup();

    WRITE_VOLATILE(core_1_ready, true);
    while (!READ_VOLATILE(core_0_ready)) {
        MEMORY_BARRIER();
        delay(1);
    }

    Serial.println("Finished initialising core 1.");
}

void loop1()
{
    // Audio app parameter processing loop
    audio_app->loop();
    delay(1);
}

