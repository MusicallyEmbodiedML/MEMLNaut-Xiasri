// #include "src/memllib/interface/InterfaceBase.hpp"
#include "display.hpp"
#include "src/memllib/audio/AudioAppBase.hpp"
#include "src/memllib/audio/AudioDriver.hpp"
#include "src/memllib/hardware/memlnaut/MEMLNaut.hpp"
#include <memory>
#include "IMLInterface.hpp"
#include "interfaceRL.hpp"
#include "src/memllib/synth/maxiPAF.hpp"
#include "hardware/structs/bus_ctrl.h"

#define APP_SRAM __not_in_flash("app")

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

class PAFSynthApp : public AudioAppBase
{
public:
    static constexpr size_t kN_Params = 21;

    PAFSynthApp() : AudioAppBase() {}

    bool euclidean(float phase, const size_t n, const size_t k, const size_t offset, const float pulseWidth)
    {
        // Euclidean function
        const float fi = phase * n;
        int i = static_cast<int>(fi);
        const float rem = fi - i;
        if (i == n)
        {
            i--;
        }
        const int idx = ((i + n - offset) * k) % n;
        return (idx < k && rem < pulseWidth) ? 1 : 0;
    }
    
    //     // NOTE: Phasor is the last arg
    //     const double phasor    = args.back().as_float();
    //     const int n            = args[0].as_int();
    //     const int k            = args[1].as_int();
    //     const int offset       = (args.size() >= 4) ? args[2].as_int() : 0;
    //     const float pulseWidth = (args.size() == 5) ? args[3].as_float() : 0.5;
    //     const float fi         = phasor * n;
    //     int i                  = static_cast<int>(fi);
    //     const float rem        = fi - i;
    //     if (i == n)
    //     {
    //         i--;
    //     }
    //     const int idx = ((i + n - offset) * k) % n;
    //     result        = Value(idx < k && rem < pulseWidth ? 1 : 0);
    
    //     return result;
    // }
    


    stereosample_t __force_inline Process(const stereosample_t x) override
    {
        float x1[1];

        // const float trig = pulse.square(1);
    
        paf0.play(x1, 1, arpFreq, arpFreq + (paf0_cf * arpFreq), paf0_bw * arpFreq, paf0_vib, paf0_vfr, paf0_shift, 0);
        float y = x1[0];

        const float freq1 = arpFreq * detune;

        paf1.play(x1, 1, freq1, freq1 + (paf1_cf * freq1), paf1_bw * freq1, paf1_vib, paf1_vfr, paf1_shift, 1);
        y += x1[0];

        const float freq2 = freq1 * detune;

        paf2.play(x1, 1, freq2, freq2 + (paf2_cf * freq2), paf2_bw * freq2, paf2_vib, paf2_vfr, paf2_shift, 1);
        y += x1[0];

        const float ph = phasorOsc.phasor(1);
        const bool euclidNewNote = euclidean(ph, 12, euclidN, 0, 0.1f);
        // y = y * 0.3f;
        // const float envamp = env.play(counter==0);
        // const float envamp = line.play(counter==0);
        if(zxdetect.onZX(euclidNewNote)) {
            envamp=0.2f;
            freqIndex++;
            if(freqIndex >= 4) {
                freqIndex = 0;
            }
            arpFreq = frequencies[freqIndex];
       }else{
            constexpr float envdec = 0.2f/9000.f;
            envamp -= envdec;
            if (envamp < 0.f) {
                envamp = 0.f;
            }
        }
        // PERIODIC_DEBUG(3000, Serial.println(y);)
        y = y * envamp* envamp;
        // counter++;
        // if(counter>=9000) {
        //     counter=0;
        //     freqIndex++;
        //     if(freqIndex >= 4) {
        //         freqIndex = 0;
        //     }
        //     arpFreq = frequencies[freqIndex];
        // }

        // PERIODIC_DEBUG(10000, {
        //     Serial.println(envamp);
        // })

        float d1 = (dl1.play(y, 3500, 0.8f) * dl1mix);
        // float d2 = (dl2.play(y, 15000, 0.8f) * dl2mix);
        y = y + d1;// + d2;
        stereosample_t ret { y, y };
        frame++;
        return ret;
    }

    void Setup(float sample_rate, std::shared_ptr<InterfaceBase> interface) override
    {
        AudioAppBase::Setup(sample_rate, interface);
        maxiSettings::sampleRate = sample_rate;
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

        env.setupAR(10,100);
        arpFreq = frequencies[0];
        // line.prepare(1.f,0.f,100.f,false);
        // line.triggerEnable(true);
        envamp=1.f;
    }

    void ProcessParams(const std::vector<float>& params) override
    {
        // // Map parameters to the synth
        // synth_.mapParameters(params);
        // //Serial.print("Params processed.");
        // paf0_freq = 50.f + (params[0] * params[0] * 1000.f);
        // paf1_freq = 50.f + (params[1] * params[1] * 1000.f);

        // paf0_cf = arpFreq + (params[2] * params[2] * arpFreq * 1.f);
        // paf1_cf = arpFreq + (params[3] * params[3] * arpFreq * 1.f);
        // paf2_cf = arpFreq + (params[4] * params[4] * arpFreq * 1.f);
        paf0_cf = (params[2] * params[2]  * 1.f);
        paf1_cf = (params[3] * params[3]  * 1.f);
        paf2_cf = (params[4] * params[4]  * 1.f);

        // paf0_bw = 5.f + (params[5] * arpFreq * 0.5f);
        // paf1_bw = 5.f + (params[6] * arpFreq * 0.5f);
        // paf2_bw = 5.f + (params[7] * arpFreq * 0.5f);
        paf0_bw = 0.1f + (params[5] * 2.f);
        paf1_bw = 0.1f + (params[6] * 2.f);
        paf2_bw = 0.1f + (params[7] * 2.f);

        paf0_vib = (params[8] * params[8] * 0.99f);
        paf1_vib = (params[9] * params[9] * 0.99f);
        paf2_vib = (params[10] * params[10] * 0.99f);

        paf0_vfr = (params[11] * params[11]* 10.f);
        paf1_vfr = (params[12] * params[12] * 10.f);
        paf2_vfr = (params[13] * params[13] * 10.f);

        paf0_shift = (params[14] * 1000.f);
        paf1_shift = (params[15] * 1000.f);
        paf2_shift = (params[16] * 1000.f);

        dl1mix = params[17] * params[17] * 0.4f;
        // dl2mix = params[18] * params[18] * 0.4f;
        detune = 1.0f + (params[18] * 0.1);
        
        euclidN = static_cast<size_t>(2 + (params[19] * 5));

        // Serial.printf("%f %f %f %f %f\n", paf0_cf,  paf0_bw, paf0_vib, paf0_vfr, paf0_shift);
        
    }

protected:

    maxiPAFOperator paf0;
    maxiPAFOperator paf1;
    maxiPAFOperator paf2;

    maxiDelayline<5000> dl1;
    maxiDelayline<15100> dl2;

    maxiOsc pulse;
    maxiEnvGen env;
    

    float frame=0;

    float paf0_freq = 100;
    float paf1_freq = 100;
    float paf2_freq = 50;

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

    float dl1mix = 0.0f;
    float dl2mix = 0.0f;

    size_t counter=0;
    const size_t nFREQs = 17;
    const float frequencies[nFREQs] = {100, 200, 400,800, 400, 800, 100,1600,100,400,100,50,1600,200,100,800,400};
    size_t freqIndex = 0;
    size_t freqOffset = 0;
    float arpFreq=50;

    maxiLine line;
    float envamp;

    float detune = 1.0;

    maxiOsc phasorOsc;
    maxiTrigger zxdetect;

    size_t euclidN=4;

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
        static APP_SRAM std::vector<String> msgs = {"Wow, incredible", "Awesome", "That's amazing", "Unbelievable+","I love it!!","More of this","Yes!!!!","A-M-A-Z-I-N-G"};
        String msg = msgs[rand() % msgs.size()];
        interface->storeExperience(1.f);
        Serial.println(msg);
        
        scr.post(msg);
    });
    MEMLNaut::Instance()->setMomA2Callback([interface] () {
        static APP_SRAM std::vector<String> msgs = {"Awful!","wtf? that sucks","Get rid of this sound","Totally shite","I hate this","Why even bother?","New sound please!","No, please no!!!","Thumbs down"};
        String msg = msgs[rand() % msgs.size()];
        interface->storeExperience(-1.f);
        Serial.println(msg);
        scr.post(msg);
    });
    MEMLNaut::Instance()->setMomB1Callback([interface] () {
        interface->randomiseTheActor();
        interface->generateAction(true);
        Serial.println("The Actor is confused");
        scr.post("Actor: i'm confused");
    });
    MEMLNaut::Instance()->setMomB2Callback([interface] () {
        interface->randomiseTheCritic();
        interface->generateAction(true);
        Serial.println("The Critic is confounded");
        scr.post("Critic: totally confounded");
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

    MEMLNaut::Instance()->setRVX1Callback([interface] (float value) {
        size_t divisor = 1 + (value * 100);
        String msg = "Optimise every " + String(divisor);
        scr.post(msg);
        interface->setOptimiseDivisor(divisor);
        Serial.println(msg);
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


struct repeating_timer APP_SRAM timerDisplay;
inline bool __not_in_flash_func(displayUpdate)(__unused struct repeating_timer *t) {
    scr.update();
    return true;
}
  
void setup()
{

    scr.setup();
    bus_ctrl_hw->priority = BUSCTRL_BUS_PRIORITY_DMA_W_BITS |
        BUSCTRL_BUS_PRIORITY_DMA_R_BITS | BUSCTRL_BUS_PRIORITY_PROC1_BITS;

    uint32_t seed = get_rosc_entropy_seed(32);
    srand(seed);

    Serial.begin(115200);
    // while (!Serial) {}
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

    scr.post("MEMLNaut: let's go!");
    add_repeating_timer_ms(-39, displayUpdate, NULL, &timerDisplay);

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
    delay(1);
}

extern "C" int getentropy (void * buffer, size_t how_many) {
    uint8_t* pBuf = (uint8_t*) buffer;
    while(how_many--) {
        uint8_t rand_val = rp2040.hwrand32() % UINT8_MAX;
        *pBuf++ = rand_val;
    }
    return 0; // return "no error". Can also do EFAULT, EIO, ENOSYS
}
