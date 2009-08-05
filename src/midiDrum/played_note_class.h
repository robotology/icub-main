#ifndef PLAYED_NOTE_CLASS
#define PLAYED_NOTE_CLASS

#include <iostream>
#include ".\\DirectMidi\\CDirectMidi.h"

class played_note_class
{
public:
	int   last_measure ;

	int   played_measure ;
	int   played_beat    ;
	int   played_grid    ;
	int   played_offset  ;

	played_note_class ();
	played_note_class (played_note_class& note);
	void increase ();
	void decrease ();
	void print ();
	void set_last_measure (int lastm);
	played_note_class get_next_sync ();
	played_note_class get_prev_sync ();
};

#endif