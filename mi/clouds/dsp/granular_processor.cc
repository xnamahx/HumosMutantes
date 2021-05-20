// Copyright 2014 Olivier Gillet.
//
// Author: Olivier Gillet (pichenettes@mutable-instruments.net)
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
// 
// See http://creativecommons.org/licenses/MIT/ for more information.
//
// -----------------------------------------------------------------------------
//
// Main processing class.

#include "clouds/dsp/granular_processor.h"

#include <cstring>

#include "stmlib/dsp/parameter_interpolator.h"
#include "stmlib/utils/buffer_allocator.h"

#include "clouds/resources.h"

namespace clouds {

#define DEFAULT_SAMPLE_RATE 48000.0f

using namespace std;
using namespace stmlib;

void GranularProcessor::Init(
    void* large_buffer, size_t large_buffer_size,
    void* small_buffer, size_t small_buffer_size) {
  buffer_[0] = large_buffer;
  buffer_[1] = small_buffer;
  buffer_size_[0] = large_buffer_size;
  buffer_size_[1] = small_buffer_size;
  
  num_channels_ = 2;
  low_fidelity_ = false;
  bypass_ = false;
  sample_rate_ = DEFAULT_SAMPLE_RATE;
  
  src_down_.Init();
  src_up_.Init();
  
  ResetFilters();
  
  previous_playback_mode_ = PLAYBACK_MODE_LAST;
  reset_buffers_ = true;
  dry_wet_ = 0.0f;
  mutable_parameters()->numgrains = 32;
}

void GranularProcessor::ResetFilters() {
  for (int32_t i = 0; i < 2; ++i) {
    lp_filter_[i].Init();
    hp_filter_[i].Init();
  }
}

void GranularProcessor::ProcessGranular(
    FloatFrame* input,
    FloatFrame* output,
    size_t size) {
  // At the exception of the spectral mode, all modes require the incoming  
  // audio signal to be written to the recording buffer.
    
    const float* input_samples = &input[0].l;
    for (int32_t i = 0; i < num_channels_; ++i) {

        buffer_16_[i].WriteFade(
            &input_samples[i], size, 2, !parameters_.freeze);
      
    }
    
      parameters_.granular.use_deterministic_seed = parameters_.density < 0.5f;
      if (parameters_.density >= 0.53f) {
        parameters_.granular.overlap = (parameters_.density - 0.53f) * 2.12f;
      } else if (parameters_.density <= 0.47f) {
        parameters_.granular.overlap = (0.47f - parameters_.density) * 2.12f;
      } else {
        parameters_.granular.overlap = 0.0f;
      }
      // And TEXTURE too.
      parameters_.granular.window_shape = parameters_.texture < 0.75f
          ? parameters_.texture * 1.333f : 1.0f;
  
      if (resolution() == 8) {
        player_.Play(buffer_8_, parameters_, &output[0].l, size);
      } else {
        player_.Play(buffer_16_, parameters_, &output[0].l, size);
      }

}

void GranularProcessor::Process(
    FloatFrame* input,
    FloatFrame* output,
    FloatFrame* ext_input,
    FloatFrame* outputf,
    size_t size) {
  // TIC
  if (bypass_) {
    copy(&input[0], &input[size], &output[0]);
    return;
  }
  
  if (silence_ || reset_buffers_ ||
      previous_playback_mode_ != playback_mode_) {
    float* output_samples = &output[0].l;
    fill(&output_samples[0], &output_samples[size << 1], 0);
      
    float* output_samplesf = &outputf[0].l;
    fill(&output_samplesf[0], &output_samplesf[size << 1], 0);
    return;
  }
  
  // Convert input buffers to float, and mixdown for mono processing.
  for (size_t i = 0; i < size; ++i) {
    in_[i].l = input[i].l;
    in_[i].r = input[i].r;
  }
  
  // Apply feedback, with high-pass filtering to prevent build-ups at very
  // low frequencies (causing large DC swings).
  ONE_POLE(freeze_lp_, parameters_.freeze ? 1.0f : 0.0f, 0.0005f)
  float feedback = parameters_.feedback;
    
    if(hpActive){
        hp_filter_[0].Process<FILTER_MODE_HIGH_PASS>(&ext_input[0].l, &ext_input[0].l, size, 2);
        hp_filter_[1].Process<FILTER_MODE_HIGH_PASS>(&ext_input[0].r, &ext_input[0].r, size, 2);
    }

    if(lpActive){
        lp_filter_[0].Process<FILTER_MODE_LOW_PASS>(&ext_input[0].l, &ext_input[0].l, size, 2);
        lp_filter_[1].Process<FILTER_MODE_LOW_PASS>(&ext_input[0].r, &ext_input[0].r, size, 2);
    }



  float fb_gain = feedback * (1.0f - freeze_lp_);
  for (size_t i = 0; i < size; ++i) {
    in_[i].l += (
        SoftLimit(fb_gain * 1.4f * ext_input[i].l));
    in_[i].r += (
        SoftLimit(fb_gain * 1.4f * ext_input[i].r));
  }

    ProcessGranular(in_, out_, size);
  
  for (size_t i = 0; i < size; ++i) {

    output[i].l = SoftLimit(out_[i].l);
    output[i].r = SoftLimit(out_[i].r);
      
    outputf[i].l = SoftLimit(fb_[i].l);
    outputf[i].r = SoftLimit(fb_[i].r);

  }
    
    // This is what is fed back. Reverb is not fed back.
    copy(&out_[0], &out_[size], &fb_[0]);
}

void GranularProcessor::PreparePersistentData() {
  persistent_state_.write_head[0] = low_fidelity_ ?
      buffer_8_[0].head() : buffer_16_[0].head();
  persistent_state_.write_head[1] = low_fidelity_ ?
      buffer_8_[1].head() : buffer_16_[1].head();
  persistent_state_.quality = quality();
  persistent_state_.spectral = playback_mode() == PLAYBACK_MODE_SPECTRAL;
}

void GranularProcessor::GetPersistentData(
      PersistentBlock* block, size_t *num_blocks) {
  PersistentBlock* first_block = block;
  
  block->tag = FourCC<'s', 't', 'a', 't'>::value;
  block->data = &persistent_state_;
  block->size = sizeof(PersistentState);
  ++block;

  // Create save block holding the audio buffers.
  for (int32_t i = 0; i < num_channels_; ++i) {
    block->tag = FourCC<'b', 'u', 'f', 'f'>::value;
    block->data = buffer_[i];
    block->size = buffer_size_[num_channels_ - 1];
    ++block;
  }
  *num_blocks = block - first_block;
}

bool GranularProcessor::LoadPersistentData(const uint32_t* data) {
  // Force a silent output while the swapping of buffers takes place.
  silence_ = true;
  
  PersistentBlock block[4];
  size_t num_blocks;
  GetPersistentData(block, &num_blocks);
  
  for (size_t i = 0; i < num_blocks; ++i) {
    // Check that the format is correct.
    if (block[i].tag != data[0] || block[i].size != data[1]) {
      silence_ = false;
      return false;
    }
    
    // All good. Load the data. 2 words have already been used for the block tag
    // and the block size.
    data += 2;
    memcpy(block[i].data, data, block[i].size);
    data += block[i].size / sizeof(uint32_t);
    
    if (i == 0) {
      // We now know from which mode the data was saved.
      bool currently_spectral = playback_mode_ == PLAYBACK_MODE_SPECTRAL;
      bool requires_spectral = persistent_state_.spectral;
      if (currently_spectral ^ requires_spectral) {
        set_playback_mode(requires_spectral
            ? PLAYBACK_MODE_SPECTRAL
            : PLAYBACK_MODE_GRANULAR);
      }
      set_quality(persistent_state_.quality);

      // We can force a switch to this mode, and once everything has been
      // initialized for this mode, we continue with the loop to copy the
      // actual buffer data - with all state variables correctly initialized.
      Prepare();
      GetPersistentData(block, &num_blocks);
    }
  }
  
  // We can finally reset the position of the write heads.
  if (low_fidelity_) {
    buffer_8_[0].Resync(persistent_state_.write_head[0]);
    buffer_8_[1].Resync(persistent_state_.write_head[1]);
  } else {
    buffer_16_[0].Resync(persistent_state_.write_head[0]);
    buffer_16_[1].Resync(persistent_state_.write_head[1]);
  }
  parameters_.freeze = true;
  silence_ = false;
  return true;
}

void GranularProcessor::Prepare() {
  bool playback_mode_changed = previous_playback_mode_ != playback_mode_;
  bool benign_change = previous_playback_mode_ != PLAYBACK_MODE_SPECTRAL
      && playback_mode_ != PLAYBACK_MODE_SPECTRAL
      && previous_playback_mode_ != PLAYBACK_MODE_LAST;
  
  if (!reset_buffers_ && playback_mode_changed && benign_change) {
    ResetFilters();
    pitch_shifter_.Clear();
    previous_playback_mode_ = playback_mode_;
  }
  
  if ((playback_mode_changed && !benign_change) || reset_buffers_) {
    parameters_.freeze = false;
  }
  
  if (reset_buffers_ || (playback_mode_changed && !benign_change)) {
    void* buffer[2];
    size_t buffer_size[2];
    void* workspace;
    size_t workspace_size;

      // Large buffer: 64k of sample memory + FX workspace.
      // small buffer: 64k of sample memory.
      buffer_size[0] = buffer_size[1] = buffer_size_[1];
      buffer[0] = buffer_[0];
      buffer[1] = buffer_[1];

      for (int32_t i = 0; i < num_channels_; ++i) {

          buffer_16_[i].Init(
              buffer[i],
              ((buffer_size[i]) >> 1),
              tail_buffer_[i]);

      }
      
      int32_t num_grains = static_cast<int>(parameters_.numgrains) * \
          (low_fidelity_ ? 23 : 16) >> 4;
      player_.Init(num_channels_, num_grains);
      //player_.Init(num_channels_,  static_cast<int>(parameters_.numgrains));

      reset_buffers_ = false;
      previous_playback_mode_ = playback_mode_;
  }
    
}

}  // namespace clouds
