#include "src/memllib/interface/InterfaceBase.hpp"
#include "src/memllib/audio/AudioAppBase.hpp"
#include "src/memllib/audio/AudioDriver.hpp"
#include "src/memllib/hardware/memlnaut/MEMLNaut.hpp"
#include <memory>

// Includes for the IML interface
#include "src/memlp/Dataset.hpp"
#include "src/memlp/MLP.h"

// Includes for FM Synth
#include "src/memllib/synth/FMSynth.hpp"


class IMLInterface : public InterfaceBase
{
public:
    IMLInterface() : InterfaceBase() {}

    void setup(size_t n_inputs, size_t n_outputs) override
    {
        InterfaceBase::setup(n_inputs, n_outputs);
        // Additional setup code specific to IMLInterface
        n_inputs_ = n_inputs;
        n_outputs_ = n_outputs;

        MLSetup_();
        n_iterations_ = 1000;
        input_state_.resize(n_inputs, 0.5f);
        output_state_.resize(n_outputs, 0);
        // Init/reset state machine
        training_mode_ = INFERENCE_MODE;
        perform_inference_ = true;
        input_updated_ = false;

        Serial.println("IMLInterface setup done");
        Serial.print("Address of n_inputs_: ");
        Serial.println(reinterpret_cast<uintptr_t>(&n_inputs_));
        Serial.print("Inputs: ");
        Serial.print(n_inputs_);
        Serial.print(", Outputs: ");
        Serial.println(n_outputs_);
    }

    enum training_mode_t {
        INFERENCE_MODE,
        TRAINING_MODE
    };

    void SetTrainingMode(training_mode_t training_mode)
    {
        Serial.print("Training mode: ");
        Serial.println(training_mode == INFERENCE_MODE ? "Inference" : "Training");

        if (training_mode == INFERENCE_MODE && training_mode_ == TRAINING_MODE) {
            // Train the network!
            MLTraining_();
        }
        training_mode_ = training_mode;
    }

    void ProcessInput()
    {
        // Check if input is updated
        if (perform_inference_ && input_updated_) {
            MLInference_(input_state_);
            input_updated_ = false;
        }
    }

    void SetInput(size_t index, float value)
    {
        // Serial.print("Input ");
        // Serial.print(index);
        // Serial.print(" set to: ");
        // Serial.println(value);

        if (index >= n_inputs_) {
            Serial.print("Input index ");
            Serial.print(index);
            Serial.println(" out of bounds.");
            return;
        }

        if (value < 0) {
            value = 0;
        } else if (value > 1.0) {
            value = 1.0;
        }

        // Update state of input
        input_state_[index] = value;
        input_updated_ = true;
    }

    enum saving_mode_t {
        STORE_VALUE_MODE,
        STORE_POSITION_MODE,
    };

    void SaveInput(saving_mode_t mode)
    {
        if (STORE_VALUE_MODE == mode) {

            Serial.println("Move input to position...");
            perform_inference_ = false;

        } else {  // STORE_POSITION_MODE

            Serial.println("Creating example in this position.");
            // Save pair in the dataset
            dataset_->Add(input_state_, output_state_);
            perform_inference_ = true;
            MLInference_(input_state_);

        }
    }

    void ClearData()
    {
        if (training_mode_ == TRAINING_MODE) {
            Serial.println("Clearing dataset...");
            dataset_->Clear();
        }
    }

    void Randomise()
    {
        if (training_mode_ == TRAINING_MODE) {
            Serial.println("Randomising weights...");
            MLRandomise_();
            MLInference_(input_state_);
        }
    }

    void SetIterations(size_t iterations)
    {
        n_iterations_ = iterations;
        Serial.print("Iterations set to: ");
        Serial.println(n_iterations_);
    }

protected:
    size_t n_inputs_;
    size_t n_outputs_;
    size_t n_iterations_;

    // State machine
    training_mode_t training_mode_;
    bool perform_inference_;
    bool input_updated_;

    // Controls/sensors
    std::vector<float> input_state_;
    std::vector<float> output_state_;

    // MLP core
    std::unique_ptr<Dataset> dataset_;
    std::unique_ptr<MLP<float>> mlp_;
    MLP<float>::mlp_weights mlp_stored_weights_;
    bool randomised_state_;

    void MLSetup_()
    {
        // Constants for MLP init
        const unsigned int kBias = 1;
        const std::vector<ACTIVATION_FUNCTIONS> layers_activfuncs = {
            RELU, RELU, RELU, SIGMOID
        };
        const bool use_constant_weight_init = false;
        const float constant_weight_init = 0;
        // Layer size definitions
        const std::vector<size_t> layers_nodes = {
            n_inputs_ + kBias,
            10, 10, 14,
            n_outputs_
        };

        // Create dataset
        dataset_ = std::make_unique<Dataset>();
        // Create MLP
        mlp_ = std::make_unique<MLP<float>>(
            layers_nodes,
            layers_activfuncs,
            loss::LOSS_MSE,
            use_constant_weight_init,
            constant_weight_init
        );

        // State machine
        randomised_state_ = false;
    }

    void MLInference_(std::vector<float> input)
    {
        if (!dataset_ || !mlp_) {
            Serial.println("ML not initialized!");
            return;
        }

        if (input.size() != n_inputs_) {
            Serial.print("Input size mismatch - ");
            Serial.print("Expected: ");
            Serial.print(n_inputs_);
            Serial.print(", Got: ");
            Serial.println(input.size());
            return;
        }

        input.push_back(1.0f); // Add bias term
        // Perform inference
        std::vector<float> output(n_outputs_);
        mlp_->GetOutput(input, &output);
        // Process inferenced data
        output_state_ = output;
        SendParamsToQueue(output);
    }

    void MLRandomise_()
    {
        if (!mlp_) {
            Serial.println("ML not initialized!");
            return;
        }

        // Randomize weights
        mlp_stored_weights_ = mlp_->GetWeights();
        mlp_->DrawWeights();
        randomised_state_ = true;
    }

    void MLTraining_()
    {
        if (!mlp_) {
            Serial.println("ML not initialized!");
            return;
        }
        // Restore old weights
        if (randomised_state_) {
            mlp_->SetWeights(mlp_stored_weights_);
        }
        randomised_state_ = false;

        // Prepare for training
        // Extract dataset to training pair
        MLP<float>::training_pair_t dataset(dataset_->GetFeatures(), dataset_->GetLabels());
        // Check and report on dataset size
        Serial.print("Feature size ");
        Serial.print(dataset.first.size());
        Serial.print(", label size ");
        Serial.println(dataset.second.size());
        if (!dataset.first.size() || !dataset.second.size()) {
            Serial.println("Empty dataset!");
            return;
        }
        Serial.print("Feature dim ");
        Serial.print(dataset.first[0].size());
        Serial.print(", label dim ");
        Serial.println(dataset.second[0].size());
        if (!dataset.first[0].size() || !dataset.second[0].size()) {
            Serial.println("Empty dataset dimensions!");
            return;
        }

        // Training loop
        Serial.print("Training for max ");
        Serial.print(n_iterations_);
        Serial.println(" iterations...");
        float loss = mlp_->Train(dataset,
                1.,
                n_iterations_,
                0.00001,
                false);
        Serial.print("Trained, loss = ");
        Serial.println(loss, 10);
    }
};

#include "src/memllib/synth/maxiPAF.hpp"

class PAFSynthApp : public AudioAppBase
{
public:
    static constexpr size_t kN_Params = 10;

    PAFSynthApp() : AudioAppBase() {}

    inline stereosample_t Process(const stereosample_t x) override
    {
        float x1[1];

        paf0.play(x1, 1, paf0_freq, paf0_cf, paf0_bw, paf0_vib, paf0_vfr, 1);
        float y = x1[0];

        // paf1.play(x1, 1, paf1_freq, paf1_cf, paf1_bw, 0, 0, 1);
        // y += x1[0];

        // paf2.play(x1, 1, paf1_freq, paf1_cf, paf1_bw, paf1_vib, paf1_vfr, 1);
        // y += x1[0];

        // paf2.play(x1, 1);
        // y += x1[0];

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
        paf0.freq(100, 0);
        // paf0.amp(1,0);
        paf0.bw(200,0);
        paf0.cf(210,0);
        paf0.vfr(5,0);
        paf0.vib(0.1,0);
        paf0.shift(10,0);

        paf1.init();
        paf1.setsr(maxiSettings::getSampleRate(), 1);
        paf1.freq(150, 0);
        // paf1.amp(1,0);
        paf1.bw(200,0);
        paf1.cf(210,0);
        paf1.vfr(5,0);
        paf1.vib(0.1,0);
        paf1.shift(10,0);

        paf2.init();
        paf2.setsr(maxiSettings::getSampleRate(), 1);
        paf2.freq(190, 0);
        // paf2.amp(1,0);
        paf2.bw(500,0);
        paf2.cf(210,0);
        paf2.vfr(5,0);
        paf2.vib(0.1,0);
        paf2.shift(6,0);
    }

    void ProcessParams(const std::vector<float>& params) override
    {
        // // Map parameters to the synth
        // synth_.mapParameters(params);
        // //Serial.print("Params processed.");
        // paf0_freq = 50.f + (params[0] * params[0] * 1000.f);
        // paf1_freq = 50.f + (params[1] * params[1] * 1000.f);

        paf0_cf = paf0_freq + (params[2] * params[2] * paf0_freq * 16.f);
        paf1_cf = 50.f + (params[3] * params[3] * 1000.f);

        paf0_bw = 10.f + (params[4] * paf0_freq);
        paf1_bw = 50.f + (params[5] * 5000.f);

        paf0_vib = (params[6] * params[6] * 0.9f);
        paf1_vib = (params[7] * params[7] * 0.9f);

        paf0_vfr = (params[8] * params[8]* 15.f);
        paf1_vfr = (params[9] * params[9] * 15.f);

        Serial.printf("%f %f %f %f\n", paf0_cf,  paf0_bw, paf0_vib, paf0_vfr);
    }

protected:

    maxiPAFOperator paf0;
    maxiPAFOperator paf1;
    maxiPAFOperator paf2;

    float frame=0;

    float paf0_freq = 100;
    float paf1_freq = 101;
    float paf0_cf = 200;
    float paf1_cf = 250;

    float paf0_bw = 100;
    float paf1_bw = 5000;

    float paf0_vib = 0;
    float paf1_vib = 1;

    float paf0_vfr = 2;
    float paf1_vfr = 2;

};




// Global objects
std::shared_ptr<IMLInterface> interface;
std::shared_ptr<PAFSynthApp> AUDIO_MEM audio_app;

// Inter-core communication
volatile bool core_0_ready = false;
volatile bool core_1_ready = false;
volatile bool serial_ready = false;
volatile bool interface_ready = false;


// We're only bound to the joystick inputs (x, y, rotate)
const size_t kN_InputParams = 3;

// Add these macros near other globals
#define MEMORY_BARRIER() __sync_synchronize()
#define WRITE_VOLATILE(var, val) do { MEMORY_BARRIER(); (var) = (val); MEMORY_BARRIER(); } while (0)
#define READ_VOLATILE(var) ({ MEMORY_BARRIER(); typeof(var) __temp = (var); MEMORY_BARRIER(); __temp; })


void bind_interface(std::shared_ptr<IMLInterface> interface)
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


void setup()
{
    Serial.begin(115200);
    while (!Serial) {}
    Serial.println("Serial initialised.");
    WRITE_VOLATILE(serial_ready, true);

    // Setup board
    MEMLNaut::Initialize();
    pinMode(33, OUTPUT);

    // Setup interface with memory barrier protection
    {
        auto temp_interface = std::make_shared<IMLInterface>();
        temp_interface->setup(kN_InputParams, PAFSynthApp::kN_Params);
        MEMORY_BARRIER();
        interface = temp_interface;
        MEMORY_BARRIER();
    }
    WRITE_VOLATILE(interface_ready, true);

    // Bind interface after ensuring it's fully initialized
    bind_interface(interface);
    Serial.println("Bound interface to MEMLNaut.");

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
    delay(20); // Add a small delay to avoid flooding the serial output
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
        temp_audio_app->Setup(AudioDriver::GetSampleRate(), interface);
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
}

extern "C" int getentropy (void * buffer, size_t how_many) {
    uint8_t* pBuf = (uint8_t*) buffer;
    while(how_many--) {
        uint8_t rand_val = rp2040.hwrand32() % UINT8_MAX;
        *pBuf++ = rand_val;
    }
    return 0; // return "no error". Can also do EFAULT, EIO, ENOSYS
}
