#include "c74_msp.h"
#include "clouds/dsp/granular_processor.h"
#include <iostream>

using namespace c74::max;

static t_class* this_class = nullptr;

inline double constrain(double v, double vMin, double vMax) {
	return std::max<double>(vMin, std::min<double>(vMax, v));
}

struct t_humos {
	t_pxobject x_obj;

	double  f_dummy;

	double f_reverse;
	double f_distort;
	double f_freeze;
	double f_trig;
	double f_position;
	double f_size;
	double f_pitch;
	double f_density;
	double f_overlap;
	double f_texture;
	double f_mix;
	double f_spread;
	double f_feedback;
	double f_lp_cutoff;
	double f_hp_cutoff;
	double f_lp_res;
	double f_hp_res;
	double f_reverb;
	double f_mode;
	double f_mono;
	double f_silence;
	double f_bypass;
	double f_lofi;
	double f_num_channels;
	double f_numgrains;
	double f_grainsize;


	clouds::GranularProcessor processor;
	clouds::FloatFrame* ibuf;
	clouds::FloatFrame* obuf;
    clouds::FloatFrame* obuff;
    
    clouds::FloatFrame* extibuf;
    
	int iobufsz = 64;
	long sr = 48000.0;

	static const int LARGE_BUF = 262144;
	static const int SMALL_BUF = 262144;
	//static const int SMALL_BUF = 65536 - 128;
	/*
262144
	static const int LARGE_BUF = 118784;
	524288
	static const int SMALL_BUF = 65536 - 128;
	*/

	uint8_t* large_buf;
	int      large_buf_size;
	uint8_t* small_buf;
	int      small_buf_size;

	void changeRate();
	void init();

};


void t_humos::init() {

	f_lp_cutoff = 0.5f;
	f_hp_cutoff = 0.5f;
	f_lp_res = 0.5f;
	f_hp_res = 0.5f;

	large_buf_size = t_humos::LARGE_BUF;
	large_buf = new uint8_t[large_buf_size];
	small_buf_size = t_humos::SMALL_BUF;
	small_buf = new uint8_t[small_buf_size];
	processor.Init(large_buf,LARGE_BUF,small_buf,SMALL_BUF);
	changeRate();
	processor.Prepare();
}

void t_humos::changeRate() {
	ibuf = new clouds::FloatFrame[iobufsz];
	obuf = new clouds::FloatFrame[iobufsz];
    obuff = new clouds::FloatFrame[iobufsz];
    extibuf = new clouds::FloatFrame[iobufsz];
}

void humos_perform64(t_humos* self, t_object* dsp64, double** ins, long numins, double** outs, long numouts, long sampleframes, long flags, void* userparam) {
	
    double    *in = ins[0];
    double    *in2 = ins[1];
    double    *in3 = ins[2];
    double    *in4 = ins[3];
    double    *out = outs[0];
    double    *out2 = outs[1];
    
    double    *out3 = outs[2];
    double    *out4 = outs[3];

	for (int i=0; i<sampleframes; ++i){
		self->ibuf[i].l = *in++;
		self->ibuf[i].r = *in2++;
        self->extibuf[i].l = *in3++;
        self->extibuf[i].r = *in4++;
	}

	self->processor.Process(self->ibuf, self->obuf, self->extibuf, self->obuff, sampleframes);

	for (int i = 0; i < sampleframes; i++) {
		*out++ = self->obuf[i].l;
        *out2++ = self->obuf[i].r;
        *out3++ = self->obuff[i].l;
        *out4++ = self->obuff[i].r;
	}
}

void* humos_new(void) {
	t_humos* self = (t_humos*)object_alloc(this_class);

	dsp_setup((t_pxobject*)self, 4);

	outlet_new(self, "signal");
	outlet_new(self, "signal");
    outlet_new(self, "signal");
    outlet_new(self, "signal");
	inlet_new(self, NULL);

	self->init();
	self->x_obj.z_misc = Z_NO_INPLACE;

	return (void *)self;
}

void humos_free(t_humos* self) {
	dsp_free((t_pxobject*)self);
}

void humos_dsp64(t_humos* self, t_object* dsp64, short* count, double samplerate, long maxvectorsize, long flags) {
	object_method_direct(void, (t_object*, t_object*, t_perfroutine64, long, void*),
						 dsp64, gensym("dsp_add64"), (t_object*)self, (t_perfroutine64)humos_perform64, 0, NULL);

	if (self->sr!=samplerate || self->iobufsz!=maxvectorsize)
	{
		self->sr=samplerate;
		self->iobufsz=maxvectorsize;
		self->changeRate();
	}
}


void humos_assist(t_humos* self, void* unused, t_assist_function io, long index, char* string_dest) {
	if (io == ASSIST_INLET) {
		switch (index) {
			case 0: 
				strncpy(string_dest,"Properties", ASSIST_STRING_MAXSIZE); 
				break;
		}
	}
	else if (io == ASSIST_OUTLET) {
		switch (index) {
			case 0: 
				strncpy(string_dest,"(signal) L Output", ASSIST_STRING_MAXSIZE); 
				break;
			case 1: 
				strncpy(string_dest,"(signal) R Output", ASSIST_STRING_MAXSIZE); 
				break;
		}
	}
}


void humos_freeze(t_humos *x, double f)
{
  	x->f_freeze = f;
	x->processor.mutable_parameters()->freeze = (x->f_freeze > 0.5f);
}
void humos_reverse(t_humos *x, double f)
{
  	x->f_reverse = f;
	x->processor.ToggleReverse(f);
}
void humos_trig(t_humos *x, double f)
{
  	x->f_trig = f;
	//note the trig input is really a gate... which then feeds the trig
	x->processor.mutable_parameters()->gate = (x->f_trig > 0.5f);
	x->processor.mutable_parameters()->trigger = (x->f_trig > 0.5f);
}
void humos_position(t_humos *x, double f)
{
	x->f_position = f;
	x->processor.mutable_parameters()->position = constrain(x->f_position, 0.0f, 1.0f);
}
void humos_distort(t_humos *x, double f)
{
	x->f_distort = f;
	x->processor.mutable_parameters()->distort = constrain(x->f_distort, 0.0f, 1.0f);
}
void humos_size(t_humos *x, double f)
{
	x->f_size = f;
	x->processor.mutable_parameters()->size = constrain(x->f_size, 0.0f, 1.0f);
}

void humos_pitch(t_humos *x, double f)
{
  	x->f_pitch = f;
	x->processor.mutable_parameters()->pitch = constrain(x->f_pitch , -24.0f, 24.0f);
}
void humos_density(t_humos *x, double f)
{
  	x->f_density = f;
	double density = constrain(x->f_density, 0.0f, 1.0f);
	density = (x->f_mode == clouds::PLAYBACK_MODE_GRANULAR) ? (density*0.6f) + 0.2f : density;
	x->processor.mutable_parameters()->density = constrain(density, 0.0f, 1.0f);
}

void humos_overlap(t_humos *x, double f)
{
  	x->f_overlap = f;
	x->processor.mutable_parameters()->overlap = constrain(x->f_overlap, 0.0f, 1.0f);
}

void humos_texture(t_humos *x, double f)
{
	x->f_texture = f;
	x->processor.mutable_parameters()->texture = constrain(x->f_texture, 0.0f, 1.0f);
}

void humos_mix(t_humos *x, double f)
{
	x->f_mix = f;
	x->processor.mutable_parameters()->dry_wet = constrain(x->f_mix, 0.0f, 1.0f);
}

void humos_spread(t_humos *x, double f)
{
  	x->f_spread = f;
	x->processor.mutable_parameters()->stereo_spread = constrain(x->f_spread, 0.0f, 1.0f);
}
void humos_numgrains(t_humos *x, double f)
{
  	x->f_numgrains = f;
	x->processor.mutable_parameters()->numgrains = x->f_numgrains;
    x->processor.reset_buffers();
    x->processor.Prepare();
}

void humos_feedback(t_humos *x, double f)
{
  	x->f_feedback = constrain(f, 0.0f, 1.2f);
	x->processor.mutable_parameters()->feedback = x->f_feedback;
};

void humos_lp_active(t_humos *x, double f)
{
    x->processor.lpActive = f > 0.5f;
}

void humos_hp_active(t_humos *x, double f)
{
    x->processor.hpActive = f > 0.5f;
}

void humos_lp_cutoff(t_humos *x, double f)
{
  	x->f_lp_cutoff = constrain(f, 0.0f, 1.0f);
	x->processor.set_lp_cutoff(x->f_lp_cutoff,x->f_lp_res);
}

void humos_lp_resonance(t_humos *x, double f)
{
  	x->f_lp_res = constrain(f, 0.05f, 0.9f);
	x->processor.set_lp_cutoff(x->f_lp_cutoff,x->f_lp_res);
}

void humos_hp_cutoff(t_humos *x, double f)
{
  	x->f_hp_cutoff = constrain(f, 0.0f, 1.0f);
	x->processor.set_hp_cutoff(x->f_hp_cutoff,x->f_hp_res);
}

void humos_hp_resonance(t_humos *x, double f)
{
  	x->f_hp_res = constrain(f, 0.05f, 0.9f);
	x->processor.set_hp_cutoff(x->f_hp_cutoff,x->f_hp_res);
}


void humos_mode(t_humos *x, double f)
{
  	x->f_mode = f;
	clouds::PlaybackMode mode = (clouds::PlaybackMode) (int(x->f_mode) % clouds::PLAYBACK_MODE_LAST);
	if (mode != x->processor.playback_mode()) {
		x->processor.set_playback_mode(mode);
		switch (mode) {
		//case clouds::PLAYBACK_MODE_humos: object_post(&x->x_obj.z_ob,"clds:humos"); x->processor.set_num_channels(2); break;
		case clouds::PLAYBACK_MODE_GRANULAR: object_post(&x->x_obj.z_ob,"clds:granular"); x->processor.set_num_channels(2); break;
		/*case clouds::PLAYBACK_MODE_STRETCH: object_post(&x->x_obj.z_ob,"clds:stretch"); x->processor.set_num_channels(2); break;
		case clouds::PLAYBACK_MODE_LOOPING_DELAY: object_post(&x->x_obj.z_ob,"clds:looping"); x->processor.set_num_channels(2); break;
		case clouds::PLAYBACK_MODE_SPECTRAL: object_post(&x->x_obj.z_ob,"clds:spectral"); x->processor.set_num_channels(1); break;
		case clouds::PLAYBACK_MODE_LAST:*/
		default: object_post(&x->x_obj.z_ob,"clds : unknown mode");
		}
	}
}

void humos_mono(t_humos *x, double f)
{
  x->f_mono = f;
}

void humos_silence(t_humos *x, double f)
{
  	x->f_silence = f;
	x->processor.set_silence(x->f_silence > 0.5f);
}

void humos_bypass(t_humos *x, double f)
{
  	x->f_bypass = f;
	x->processor.set_bypass(x->f_bypass > 0.5f);
}

void humos_lofi(t_humos *x, double f)
{
  	x->f_lofi = f;
	x->processor.set_low_fidelity(x->f_lofi > 0.5f);
	x->processor.Prepare();
}

void humos_numchannels(t_humos *x, double f)
{
  	x->f_num_channels = f;
	x->processor.set_num_channels(x->f_num_channels);
	x->processor.Prepare();
}

void humos_samplerate(t_humos *x, double f)
{
	x->processor.sample_rate(f);
	x->processor.Prepare();
}

void humos_grainsize(t_humos *x, double f)
{
	x->f_grainsize = f * 8192.0f;
	x->processor.mutable_parameters()->grain_size_hint = constrain(x->f_grainsize, 128.0f, 8192.0f);
}	

void ext_main(void* r) {
	this_class = class_new("humos~", (method)humos_new, (method)humos_free, sizeof(t_humos), NULL, A_GIMME, 0);

	class_addmethod(this_class,(method) humos_assist, "assist",	A_CANT,		0);
	class_addmethod(this_class,(method) humos_dsp64, "dsp64",	A_CANT,		0);
	
	class_addmethod(this_class,(method) humos_freeze, "freeze", A_DEFFLOAT, 0);
	class_addmethod(this_class,(method) humos_reverse, "reverse", A_DEFFLOAT, 0);
	class_addmethod(this_class,(method) humos_trig, "trig", A_DEFFLOAT, 0);
	class_addmethod(this_class,(method) humos_distort, "distort", A_DEFFLOAT, 0);
	class_addmethod(this_class,(method) humos_position, "position", A_DEFFLOAT, 0);
	class_addmethod(this_class,(method) humos_size, "size", A_DEFFLOAT, 0);
	class_addmethod(this_class,(method) humos_numgrains, "numgrains", A_DEFFLOAT, 0);
	class_addmethod(this_class,(method) humos_grainsize, "grainsize", A_DEFFLOAT, 0);

	class_addmethod(this_class,(method) humos_pitch, "pitch", A_DEFFLOAT, 0);
	class_addmethod(this_class,(method) humos_density, "density", A_DEFFLOAT, 0);
	class_addmethod(this_class,(method) humos_overlap, "overlap", A_DEFFLOAT, 0);
	class_addmethod(this_class,(method) humos_texture, "texture", A_DEFFLOAT, 0);
	class_addmethod(this_class,(method) humos_mix, "mix", A_DEFFLOAT, 0);
	class_addmethod(this_class,(method) humos_spread, "spread", A_DEFFLOAT, 0);
	class_addmethod(this_class,(method) humos_feedback, "feedback", A_DEFFLOAT, 0);

	class_addmethod(this_class,(method) humos_lp_cutoff, "lp_cutoff", A_DEFFLOAT, 0);
	class_addmethod(this_class,(method) humos_hp_cutoff, "hp_cutoff", A_DEFFLOAT, 0);
	class_addmethod(this_class,(method) humos_lp_resonance, "lp_resonance", A_DEFFLOAT, 0);
	class_addmethod(this_class,(method) humos_hp_resonance, "hp_resonance", A_DEFFLOAT, 0);

	class_addmethod(this_class,(method) humos_mode, "mode", A_DEFFLOAT, 0);
	class_addmethod(this_class,(method) humos_mono, "mono", A_DEFFLOAT, 0);
	class_addmethod(this_class,(method) humos_silence, "silence", A_DEFFLOAT, 0);
	class_addmethod(this_class,(method) humos_bypass, "bypass", A_DEFFLOAT, 0);
	class_addmethod(this_class,(method) humos_lofi, "lofi", A_DEFFLOAT, 0);
	class_addmethod(this_class,(method) humos_samplerate, "samplerate", A_DEFFLOAT, 0);
	class_addmethod(this_class,(method) humos_numchannels, "numchannels", A_DEFFLOAT, 0);
    
    class_addmethod(this_class,(method) humos_lp_active, "humos_lp_active", A_DEFFLOAT, 0);
    class_addmethod(this_class,(method) humos_hp_active, "humos_hp_active", A_DEFFLOAT, 0);

	class_dspinit(this_class);
	class_register(CLASS_BOX, this_class);
}
