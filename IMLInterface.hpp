#ifndef IMINTERFACE_HPP
#define IMINTERFACE_HPP

// #include "src/memllib/audio/AudioAppBase.hpp"
#include "src/memllib/interface/InterfaceBase.hpp"

#include "src/memlp/Dataset.hpp"
#include "src/memlp/MLP.h"



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

#endif // IMINTERFACE_HPP
