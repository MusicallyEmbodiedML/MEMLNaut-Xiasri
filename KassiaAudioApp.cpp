#include "KassiaAudioApp.hpp"
#include "src/memllib/utils/sharedMem.hpp" // For WRITE_VOLATILE if uncommented in Process
#include <cmath> // For tanhf

stereosample_t KassiaAudioApp::Process(const stereosample_t x)
{
    float mix = x.L + x.R;
    // float bpf1Val = bpf1.play(mix) * 100.f;
    // bpf1Val = bpfEnv1.play(bpf1Val);
    // WRITE_VOLATILE(sharedMem::f0, bpf1Val);

    // float bpf2Val = bpf2.play(mix) * 100.f;
    // bpf2Val = bpfEnv2.play(bpf2Val);
    // WRITE_VOLATILE(sharedMem::f1, bpf2Val);

    // float bpf3Val = bpf3.play(mix) * 100.f;
    // bpf3Val = bpfEnv3.play(bpf3Val);
    // WRITE_VOLATILE(sharedMem::f2, bpf3Val);

    // float bpf4Val = bpf4.play(mix) * 100.f;
    // bpf4Val = bpfEnv4.play(bpf4Val);
    // WRITE_VOLATILE(sharedMem::f3, bpf4Val);

    smoother.Process(neuralNetOutputs.data(), smoothParams.data());

    // Process smoothed params
    dl1mix = smoothParams[0] * smoothParams[0] * 0.4f;
    dl2mix = smoothParams[1] * smoothParams[1] * 0.4f;
    dl3mix = smoothParams[2] * smoothParams[2] * 0.8f;
    allp1fb = smoothParams[4] * 0.99f;
    allp2fb = smoothParams[5] * 0.99f;
    float comb1fb = (smoothParams[6] * 0.95f);
    float comb2fb = (smoothParams[7] * 0.95f);

    float dl1fb = (smoothParams[8] * 0.95f);
    float dl2fb = (smoothParams[9] * 0.95f);
    float dl3fb = (smoothParams[10] * 0.95f);
    // Wet-dry mix between 0.2 and 1
    wetdry_mix_ = (smoothParams[11] * 0.8f) + 0.2f;
    // Pitch shift transposition between -12 and +12 semitones
    float pitchshift_transpose = (smoothParams[12] * 24.f) - 12.f; // Scale to -12 to +12 semitones
    pitchshifter_.SetTransposition(pitchshift_transpose);
    // Set pitch shifter mix
    pitchshifter_mix_ = smoothParams[13] * 0.99f;

    // PROCESS
    float pitchshifted = pitchshifter_.Process(mix);
    // Mix the original signal with the pitch-shifted signal
    pitchshifted = (mix * (1.f - pitchshifter_mix_)) + (pitchshifted * pitchshifter_mix_);
    float y = dcb.play(pitchshifted, 0.99f) * 3.f;
    float y1 = allp1.allpass(y, 30, allp1fb);
    y1 = comb1.combfb(y1, 127, comb1fb);

    float y2 = allp2.allpass(y, 500, allp2fb);
    y2 = comb2.combfb(y2, 808, comb2fb);

    y = y1 + y2;
    float d1 = (dl1.play(y, 3500, dl1fb) * dl1mix);
    float d2 = (dl2.play(y, 9000, dl2fb) * dl2mix);
    float d3 = (dl3.play(y, 1199, dl3fb) * dl3mix);


    y = y + d1 + d2 + d3;

    // Mix dry
    y = (y * wetdry_mix_) + (mix * (1.f - wetdry_mix_));

    y = tanhf(y);

    stereosample_t ret { y, y };

    frame++;

    return ret;
}

void KassiaAudioApp::Setup(float sample_rate, std::shared_ptr<InterfaceBase> interface)
{
    AudioAppBase::Setup(sample_rate, interface);
    maxiSettings::sampleRate = sample_rate;
    // bpf1.set(maxiBiquad::filterTypes::BANDPASS, 100.f, 5.f, 0.f);
    // bpf2.set(maxiBiquad::filterTypes::BANDPASS, 500.f, 5.f, 0.f);
    // bpf3.set(maxiBiquad::filterTypes::BANDPASS, 1000.f, 5.f, 0.f);
    // bpf4.set(maxiBiquad::filterTypes::BANDPASS, 4000.f, 5.f, 0.f);
    // Init pitch shifter
    pitchshifter_.Init(sample_rate);
}

void KassiaAudioApp::ProcessParams(const std::vector<float>& params)
{
    neuralNetOutputs = params;
}
