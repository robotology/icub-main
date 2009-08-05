#include "played_note_class.h"

using namespace std;

played_note_class::played_note_class ()
{
	played_measure = 0;
    played_beat    = 0;
    played_grid    = 0;
    played_offset  = 0;
}

played_note_class::played_note_class (played_note_class& note)
{
	played_measure = note.played_measure;
	played_beat    = note.played_beat;
	played_grid    = note.played_grid;
	played_offset  = note.played_offset;
	last_measure   = note.last_measure;
}

void played_note_class::increase ()
{
	played_measure += 0;
    played_beat    += 1;
    played_grid    += 0;
    played_offset  += 0;

	if (played_beat>=4)
	{
		played_beat =0;
		played_measure++;
	}
	if (played_measure>=last_measure)
	{
		played_measure=0;
	}
}

void played_note_class::decrease ()
{
	played_measure -= 0;
    played_beat    -= 2;
    played_grid    -= 0;
    played_offset  -= 0;

	if (played_beat<0)
	{
		played_beat =0;
		played_measure--;
	}
	if (played_measure<0)
	{
		played_measure=0;
	}
}

played_note_class played_note_class::get_next_sync ()
{
	played_note_class next_sync(*this);
	next_sync.increase();
	next_sync.played_grid=0;
	next_sync.played_offset=0;
	return next_sync;
}

played_note_class played_note_class::get_prev_sync ()
{
	played_note_class prev_sync(*this);
	prev_sync.decrease();
	prev_sync.played_grid=0;
	prev_sync.played_offset=0;
	return prev_sync;
}

void played_note_class::print ()
{
	cout << "played meas:";
	cout << int(played_measure) << " ";
	cout << int(played_beat) << " ";
	cout << int(played_grid) << " ";
	cout << int(played_offset) << endl;
}

void played_note_class::set_last_measure (int lastm)
{
	last_measure=lastm;
}