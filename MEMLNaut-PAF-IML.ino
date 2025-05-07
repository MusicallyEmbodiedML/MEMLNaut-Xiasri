// #include "src/memllib/interface/InterfaceBase.hpp"
#include "src/memllib/audio/AudioAppBase.hpp"
#include "src/memllib/audio/AudioDriver.hpp"
#include "src/memllib/hardware/memlnaut/MEMLNaut.hpp"
#include <memory>
#include "IMLInterface.hpp"
#include "interfaceRL.hpp"
#include "src/memllib/synth/maxiPAF.hpp"
#include "hardware/structs/bus_ctrl.h"

#define APP_SRAM __not_in_flash("app")



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

class PAFSynthApp : public AudioAppBase
{
public:
    static constexpr size_t kN_Params = 17;

    PAFSynthApp() : AudioAppBase() {}

    stereosample_t __force_inline Process(const stereosample_t x) override
    {
        float x1[1];

        paf0.play(x1, 1, paf0_freq, paf0_cf, paf0_bw, paf0_vib, paf0_vfr, paf0_shift, 0);
        float y = x1[0];

        paf1.play(x1, 1, paf1_freq, paf1_cf, paf1_bw, paf1_vib, paf1_vfr, paf1_shift, 1);
        y += x1[0];

        paf2.play(x1, 1, paf2_freq, paf2_cf, paf2_bw, paf2_vib, paf2_vfr, paf2_shift, 1);
        y += x1[0];


        y = y * 0.3f;

        stereosample_t ret { y, y };
        frame++;
        return ret;
    }

    void Setup(float sample_rate, std::shared_ptr<InterfaceBase> interface) override
    {
        AudioAppBase::Setup(sample_rate, interface);
        paf0.init();
        paf0.setsr(maxiSettings::getSampleRate(), 1);
        // paf0.freq(100, 0);
        // // paf0.amp(1,0);
        // paf0.bw(200,0);
        // paf0.cf(210,0);
        // paf0.vfr(5,0);
        // paf0.vib(0.1,0);
        // paf0.shift(10,0);

        paf1.init();
        paf1.setsr(maxiSettings::getSampleRate(), 1);
        // paf1.freq(150, 0);
        // // paf1.amp(1,0);
        // paf1.bw(200,0);
        // paf1.cf(210,0);
        // paf1.vfr(5,0);
        // paf1.vib(0.1,0);
        // paf1.shift(10,0);

        paf2.init();
        paf2.setsr(maxiSettings::getSampleRate(), 1);
        // paf2.freq(190, 0);
        // // paf2.amp(1,0);
        // paf2.bw(500,0);
        // paf2.cf(210,0);
        // paf2.vfr(5,0);
        // paf2.vib(0.1,0);
        // paf2.shift(6,0);
    }

    void ProcessParams(const std::vector<float>& params) override
    {
        // // Map parameters to the synth
        // synth_.mapParameters(params);
        // //Serial.print("Params processed.");
        // paf0_freq = 50.f + (params[0] * params[0] * 1000.f);
        // paf1_freq = 50.f + (params[1] * params[1] * 1000.f);

        paf0_cf = paf0_freq + (params[2] * params[2] * paf0_freq * 4.f);
        paf1_cf = paf0_freq + (params[3] * params[3] * paf1_freq * 16.f);
        paf2_cf = paf0_freq + (params[4] * params[4] * paf2_freq * 16.f);

        paf0_bw = 10.f + (params[5] * paf0_freq);
        paf1_bw = 10.f + (params[6] * paf1_freq);
        paf2_bw = 10.f + (params[7] * paf2_freq);

        paf0_vib = (params[8] * params[8] * 0.99f);
        paf1_vib = (params[9] * params[9] * 0.99f);
        paf2_vib = (params[10] * params[10] * 0.99f);

        paf0_vfr = (params[11] * params[11]* 15.f);
        paf1_vfr = (params[12] * params[12] * 15.f);
        paf2_vfr = (params[13] * params[13] * 15.f);

        paf0_shift = (params[14] * 1000.f);
        paf1_shift = (params[15] * 1000.f);
        paf2_shift = (params[16] * 1000.f);

        // Serial.printf("%f %f %f %f %f\n", paf0_cf,  paf0_bw, paf0_vib, paf0_vfr, paf0_shift);
        
    }

protected:

    maxiPAFOperator paf0;
    maxiPAFOperator paf1;
    maxiPAFOperator paf2;

    float frame=0;

    float paf0_freq = 100;
    float paf1_freq = 101;
    float paf2_freq = 102;

    float paf0_cf = 200;
    float paf1_cf = 250;
    float paf2_cf = 250;

    float paf0_bw = 100;
    float paf1_bw = 5000;
    float paf2_bw = 5000;

    float paf0_vib = 0;
    float paf1_vib = 1;
    float paf2_vib = 1;

    float paf0_vfr = 2;
    float paf1_vfr = 2;
    float paf2_vfr = 2;

    float paf0_shift = 0;
    float paf1_shift = 0;
    float paf2_shift = 0;
};




// Global objects
std::shared_ptr<IMLInterface> APP_SRAM interfaceIML;
std::shared_ptr<interfaceRL> APP_SRAM RLInterface;

std::shared_ptr<PAFSynthApp> __scratch_y("audio") audio_app;

// Inter-core communication
volatile bool APP_SRAM core_0_ready = false;
volatile bool APP_SRAM core_1_ready = false;
volatile bool APP_SRAM serial_ready = false;
volatile bool APP_SRAM interface_ready = false;


// We're only bound to the joystick inputs (x, y, rotate)
constexpr size_t kN_InputParams = 3;

// Add these macros near other globals
#define MEMORY_BARRIER() __sync_synchronize()
#define WRITE_VOLATILE(var, val) do { MEMORY_BARRIER(); (var) = (val); MEMORY_BARRIER(); } while (0)
#define READ_VOLATILE(var) ({ MEMORY_BARRIER(); typeof(var) __temp = (var); MEMORY_BARRIER(); __temp; })


void bind_RL_interface(std::shared_ptr<interfaceRL> interface)
{
    // Set up momentary switch callbacks
    MEMLNaut::Instance()->setMomA1Callback([interface] () {
        interface->storeExperience(1.f);
        Serial.println("Incredible");
    });
    MEMLNaut::Instance()->setMomA2Callback([interface] () {
        interface->storeExperience(-1.f);
        Serial.println("That sucks");
    });
    MEMLNaut::Instance()->setMomB1Callback([interface] () {
        interface->randomiseTheActor();
        Serial.println("The Actor is confused");
    });
    MEMLNaut::Instance()->setMomB2Callback([interface] () {
        interface->randomiseTheCritic();
        Serial.println("The Critic is confounded");
    });
    // Set up ADC callbacks
    MEMLNaut::Instance()->setJoyXCallback([interface] (float value) {
        interface->setState(0, value);
    });
    MEMLNaut::Instance()->setJoyYCallback([interface] (float value) {
        interface->setState(1, value);
    });
    MEMLNaut::Instance()->setJoyZCallback([interface] (float value) {
        interface->setState(2, value);
    });

    MEMLNaut::Instance()->setRVGain1Callback([interface] (float value) {
        AudioDriver::setDACVolume(value);
    });

    // Set up loop callback
    MEMLNaut::Instance()->setLoopCallback([interface] () {
        interface->optimiseSometimes();
        interface->generateAction();
    });

  
}

void bind_IML_interface(std::shared_ptr<IMLInterface> interface)
{
    // Set up momentary switch callbacks
    MEMLNaut::Instance()->setMomA1Callback([interface] () {
        interface->Randomise();
    });
    MEMLNaut::Instance()->setMomA2Callback([interface] () {
        interface->ClearData();
    });

    // Set up toggle switch callbacks
    MEMLNaut::Instance()->setTogA1Callback([interface] (bool state) {
        interface->SetTrainingMode(state ? IMLInterface::TRAINING_MODE : IMLInterface::INFERENCE_MODE);
    });
    MEMLNaut::Instance()->setJoySWCallback([interface] (bool state) {
        interface->SaveInput(state ? IMLInterface::STORE_VALUE_MODE : IMLInterface::STORE_POSITION_MODE);
    });

    // Set up ADC callbacks
    MEMLNaut::Instance()->setJoyXCallback([interface] (float value) {
        interface->SetInput(0, value);
    });
    MEMLNaut::Instance()->setJoyYCallback([interface] (float value) {
        interface->SetInput(1, value);
    });
    MEMLNaut::Instance()->setJoyZCallback([interface] (float value) {
        interface->SetInput(2, value);
    });
    MEMLNaut::Instance()->setRVZ1Callback([interface] (float value) {
        // Scale value from 0-1 range to 1-3000
        value = 1.0f + (value * 2999.0f);
        interface->SetIterations(static_cast<size_t>(value));
    });

    // Set up loop callback
    MEMLNaut::Instance()->setLoopCallback([interface] () {
        interface->ProcessInput();
    });

    MEMLNaut::Instance()->setRVGain1Callback([interface] (float value) {
        AudioDriver::setDACVolume(value);
    });
}

enum MLMODES {IML, RL};  
MLMODES APP_SRAM mlMode = RL;

void setup()
{


    bus_ctrl_hw->priority = BUSCTRL_BUS_PRIORITY_DMA_W_BITS |
        BUSCTRL_BUS_PRIORITY_DMA_R_BITS | BUSCTRL_BUS_PRIORITY_PROC1_BITS;

    uint32_t seed = get_rosc_entropy_seed(32);
    srand(seed);

    Serial.begin(115200);
    while (!Serial) {}
    Serial.println("Serial initialised.");
    WRITE_VOLATILE(serial_ready, true);

    // Setup board
    MEMLNaut::Initialize();
    pinMode(33, OUTPUT);

    switch(mlMode) {
        case IML: {
            {
                auto temp_interface = std::make_shared<IMLInterface>();
                temp_interface->setup(kN_InputParams, PAFSynthApp::kN_Params);
                MEMORY_BARRIER();
                interfaceIML = temp_interface;
                MEMORY_BARRIER();
            }
            // Setup interface with memory barrier protection
            WRITE_VOLATILE(interface_ready, true);
            // Bind interface after ensuring it's fully initialized
            bind_IML_interface(interfaceIML);
            Serial.println("Bound IML interface to MEMLNaut.");
        }
        break;
        case RL: {
            {
                auto temp_interface = std::make_shared<interfaceRL>();
                temp_interface->setup(kN_InputParams, PAFSynthApp::kN_Params);
                MEMORY_BARRIER();
                RLInterface = temp_interface;
                MEMORY_BARRIER();
            }
            // Setup interface with memory barrier protection
            WRITE_VOLATILE(interface_ready, true);
            // Bind interface after ensuring it's fully initialized
            bind_RL_interface(RLInterface);
            Serial.println("Bound RL interface to MEMLNaut.");
        }
        break;
    }


    WRITE_VOLATILE(core_0_ready, true);
    while (!READ_VOLATILE(core_1_ready)) {
        MEMORY_BARRIER();
        delay(1);
    }

    Serial.println("Finished initialising core 0.");
}

void loop()
{
        

    MEMLNaut::Instance()->loop();
    static int AUDIO_MEM blip_counter = 0;
    if (blip_counter++ > 100) {
        blip_counter = 0;
        Serial.println(".");
        // Blink LED
        digitalWrite(33, HIGH);
    } else {
        // Un-blink LED
        digitalWrite(33, LOW);
    }
    delay(10); // Add a small delay to avoid flooding the serial output
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
        auto temp_audio_app = std::make_shared<PAFSynthApp>();
        std::shared_ptr<InterfaceBase> selectedInterface;

        if (mlMode == IML) {
            selectedInterface = std::dynamic_pointer_cast<InterfaceBase>(interfaceIML);
        } else {
            selectedInterface = std::dynamic_pointer_cast<InterfaceBase>(RLInterface);
        }

        temp_audio_app->Setup(AudioDriver::GetSampleRate(), selectedInterface);
        // temp_audio_app->Setup(AudioDriver::GetSampleRate(), dynamic_cast<std::shared_ptr<InterfaceBase>> (mlMode == IML ? interfaceIML : RLInterface));
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
    delay(10);
}

extern "C" int getentropy (void * buffer, size_t how_many) {
    uint8_t* pBuf = (uint8_t*) buffer;
    while(how_many--) {
        uint8_t rand_val = rp2040.hwrand32() % UINT8_MAX;
        *pBuf++ = rand_val;
    }
    return 0; // return "no error". Can also do EFAULT, EIO, ENOSYS
}
