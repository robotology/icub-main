// Main headers

// ANSI I/0 headers
#include <conio.h>
#include <iostream>
#include <math.h>
#include <time.h>


// YARP HEADERS
#include <yarp/os/Time.h>
#include <yarp/os/all.h>
#include <yarp/os/RateThread.h>

//#include <yarp/String.h>
// The class wrapper
#include ".\\DirectMidi\\CDirectMidi.h"
#include "played_note_class.h"
// Inline library inclusion

#pragma comment (lib,"dxguid.lib") // guid definitions 
#pragma comment (lib,"winmm.lib")
#pragma comment (lib,"dsound.lib")
#pragma comment (lib,"dxerr9.lib")

using namespace std;
using namespace directmidi;             
using namespace yarp::os;

//const int SENDER_RATE=30;
const int SENDER_RATE=3000;

//#include "ppEventDebugger.h"

//static bool initialized=false;
//static ppEventDebugger ppdebugger;

class Sender: public RateThread
{
	Port port;
	int command;
	int channel;
	int note;
	int velocity;
	bool newMessage;
	Bottle bottle;
	Semaphore mutex;
	
public:
	Sender():RateThread(SENDER_RATE)
	{
		newMessage=false;
	}


	void notify(int cmd, int ch, int n, int vel)
	{
		mutex.wait();
		command=cmd;
		channel=ch;
		velocity=vel;
		note=n;

		bottle.clear();
		bottle.addInt(command);
		bottle.addInt(channel);
		bottle.addInt(note);
		bottle.addInt(velocity);

		//ppdebugger.set();
		port.write(bottle);		
		//ppdebugger.reset();
		mutex.post();
	}
	
	bool open(const char *name)
	{
		return port.open(name);
	}

	void close()
	{
		port.close();
	}

	void run()
	{
		mutex.wait();
    	//bottle.clear();
		//bottle.addInt(0);
		//port.write(bottle);		
		mutex.post();
	}


};

// Maximum size for SysEx data in input port

const int SYSTEM_EXCLUSIVE_MEM = 48000;

// Defines PI

const double PI = 3.1415926;

// Derived class from CReceiver

class CDMReceiver:public CReceiver
{
public:
	// Overriden functions
	void RecvMidiMsg(REFERENCE_TIME rt,DWORD dwChannel,DWORD dwBytesRead,BYTE *lpBuffer);
	void RecvMidiMsg(REFERENCE_TIME rt,DWORD dwChannel,DWORD dwMsg);
};

DWORD total = 0;

// Overriden function for SysEx data capture

void CDMReceiver::RecvMidiMsg(REFERENCE_TIME lprt,DWORD dwChannel,DWORD dwBytesRead,BYTE *lpBuffer)
{
	DWORD dwBytecount;
	
	// Print the received buffer
	for (dwBytecount = 0;dwBytecount < dwBytesRead;dwBytecount++)
	{	
		cout.width(2);
		cout.precision(2);
		cout.fill('0');
		
		cout << hex <<static_cast<int>(lpBuffer[dwBytecount])<< " ";
		
		if ((dwBytecount % 20) == 0) cout << endl;		
	
		if (lpBuffer[dwBytecount] == END_SYS_EX)
			cout << "\nSystem memory dumped" << endl;
			
	}	

}

#define NOTE_CRASH   49
#define NOTE_TOM1    48
#define NOTE_HIHAT   46
#define NOTE_TOM2    47
#define NOTE_TOM3    43
#define NOTE_KICK    33
#define NOTE_RIDE    51
#define NOTE_SNARE   31
//different values of pressure on the hi-hat control
//can generate two different notes (aftertouch?)
#define NOTE_KICK_H  44
#define NOTE_KICK_H2 85

#define SOUND_CRASH   49
#define SOUND_TOM1    48
#define SOUND_HIHAT   46
#define SOUND_TOM2    47
#define SOUND_TOM3    43
#define SOUND_KICK    36
#define SOUND_RIDE    51
#define SOUND_SNARE   38
//different values of pressure on the hi-hat control
//can generate two different notes (aftertouch?)
#define SOUND_KICK_H  35
#define SOUND_KICK_H2 85

//********************************************************************************
//GLOBALS
COutputPort				OutPort,OutPort2;
CPortPerformance		PortPerformance;
const int               max_seg = 6;
int                     curr_seg = 0;
CSegment				segment[max_seg];
Sender					*sender;
BufferedPort<Bottle>    yarpPortIn;
BufferedPort<Bottle>    yarpPortIn2;
bool				    start_required=false;
bool					stop_required=false;
double                  scoreGUIfreq,old_scoreGUIfreq=0;
double                  scoreGUIbpm,old_scoreGUIbpm=0;

struct note_timer_s
{
	 clock_t Begin;
	 clock_t End;
	 unsigned int counter;
	 note_timer_s();
};
note_timer_s::note_timer_s()
{
	counter = 0;
}

// filter 
clock_t last_crash;
clock_t last_tom1;
clock_t last_hihat;
clock_t last_tom2;
clock_t last_tom3;
clock_t last_kick;
clock_t last_ride;
clock_t last_snare;
clock_t last_kick_h;
clock_t last_kick_h2;

int dur_crash[4];
int dur_tom1[4];
int dur_tom2[4];
int dur_tom3[4];
int dur_hihat[4];
int dur_kick[4];
int dur_ride[4];
int dur_snare[4];
int dur_kick_h[4];
int dur_kick_h2[4];

//watch dog
clock_t  WatchBegin;
clock_t  loopback;

note_timer_s note_timer1;
note_timer_s note_timer2;
double quarter_factor=1.0;
int    quarter_factor_m=quarter_factor;
double measure_lenght=2000;  //in milliseconds
double m_eta = 10; //in milliseconds
double loaded_midi_tempo[max_seg];
Bottle output_bottle;
Bottle input_bottle;

// tempo autoseek variables
WORD wMeasure = 0;
BYTE bBeat = 0;
BYTE bGrid = 0;
short nOffset = 0;
DMUS_TIMESIGNATURE pTimeSig;
MUSIC_TIME mtStart;
MUSIC_TIME mtLen;

int elapTicks ;
double elapMilli = elapTicks/1000;
double bpm = 60000/ elapMilli;
double Hz = bpm/60;
double tTime = bpm/120;

int last_measure = 0;
int sync_beat_advance = 2;
#define MAX_MEASURE_DIFF 8

bool playback_enabled = true;
bool hit_filter_enabled=true;

played_note_class played_note;

//********************************************************************************

void SyncronizePlayback3()
{
	MUSIC_TIME		curr_music_time=0;
	MUSIC_TIME		curr_start_point=0;
	MUSIC_TIME		curr_start_time=0;
	REFERENCE_TIME  curr_ref_time=0;
	
	WORD            curr_measure, next_measure, prev_measure = 0;
	BYTE            curr_beat   , next_beat,    prev_beat    = 0;
	BYTE            curr_grid   , next_grid,    prev_grid    = 0;
	short           curr_offset , next_offset,  prev_offset  = 0;
	WORD            curr_measure2 = 0;
	BYTE            curr_beat2    = 0;
	BYTE            curr_grid2    = 0;
	short           curr_offset2  = 0;

	if (PortPerformance.IsPlaying(segment[curr_seg])==S_FALSE)
	{
		if (playback_enabled==true )
			{
				PortPerformance.PlaySegment(segment[curr_seg]);
	//			CSegment1.GetStartPoint(&curr_start_point);
		//		CSegment1.GetStartTime(&curr_start_time);			
			}
		else 
			return;
	}
	if (PortPerformance.IsPlaying(segment[curr_seg])==S_OK)
	{
		segment[curr_seg].GetStartPoint(&curr_start_point);
		segment[curr_seg].GetStartTime(&curr_start_time);
	}

	PortPerformance.GetTime(&curr_ref_time,&curr_music_time);
	MUSIC_TIME curr_time_offset = curr_music_time - (curr_start_time - curr_start_point);
	PortPerformance.TimeToRhythm(curr_time_offset,&pTimeSig,&curr_measure2,&curr_beat2,&curr_grid2,&curr_offset2);

	cout << "Curr offset ";
	cout << int(curr_measure2) << " ";
	if (curr_measure2>=last_measure)
	{
	//	curr_measure2=curr_measure2%last_measure;
		curr_measure2=0;
		cout << "(" << curr_measure2 << ")";
	}
	cout << int(curr_beat2) << " ";
	cout << int(curr_grid2) << " ";
	cout << int(curr_offset2) << endl;
	
	cout << "expected offset ";
	cout << int(played_note.played_measure) << " ";
	cout << int(played_note.played_beat) << " ";
	cout << int(played_note.played_grid) << " ";
	cout << int(played_note.played_offset) << endl;

	if ((curr_measure2-played_note.played_measure)>MAX_MEASURE_DIFF)
	{
		cout << "*** WARNING: current played point differs of more than 2 measures from expected sync point" << endl;
		cout << "jumping to synchronize..." << endl;
		played_note.played_measure=curr_measure2;
		/*while ((long int(curr_measure2)-long int(played_note.played_measure))>MAX_MEASURE_DIFF)
		{
			played_note.played_measure++;
		}
		*/
		cout << "adjusted offset ";
		cout << int(played_note.played_measure) << " ";
		cout << int(played_note.played_beat) << " ";
		cout << int(played_note.played_grid) << " ";
		cout << int(played_note.played_offset) << endl;
	}

	if ((curr_measure2-played_note.played_measure)<-MAX_MEASURE_DIFF)
	{
		cout << "*** WARNING: current played point differs of more than 2 measures from expected sync point" << endl;
		cout << "jumping to synchronize..." << endl;
		played_note.played_measure=curr_measure2;
		/*while ((long int(curr_measure2)-long int(played_note.played_measure))<-MAX_MEASURE_DIFF)
		{
			played_note.played_measure--;
		}*/
		cout << "adjusted offset ";
		cout << int(played_note.played_measure) << " ";
		cout << int(played_note.played_beat) << " ";
		cout << int(played_note.played_grid) << " ";
		cout << int(played_note.played_offset) << endl;
	}

	MUSIC_TIME exp_time_offset = 0;
	PortPerformance.RhythmToTime(played_note.played_measure,played_note.played_beat,played_note.played_grid,played_note.played_offset,&pTimeSig,&exp_time_offset);
	
	static long int time_error_old=0;
	static long int time_error=0;
	long int        time_error_dt=0;

	time_error_old = time_error;
	time_error     = exp_time_offset-curr_time_offset;
	time_error_dt  = time_error-time_error_old;

	double control_time = double(time_error)*0.0002;
//	control_time = tTime+control_time;   //@@@@ check me, is this really useful??

	if (control_time>100) control_time=100;
	if (control_time<0.01) control_time=0.01;

/*
	if (control_time>1.11) control_time=1.11;
	if (control_time<0.90) control_time=0.90;
*/
	if (abs(bpm-(control_time*loaded_midi_tempo[curr_seg]))>60)
	{
		cout << "*** WARNING: current tempo differs of more than 60bmp from expected tempo" << endl;
		cout << "setting a new tempo..." << endl;
		//PortPerformance.SetMasterTempo(double(control_time));
	}

	cout<<"current drum tempo:"<<tTime<<" (extimated bmp: "<<bpm<<")"<< endl;
	cout<<"setting song tempo:"<<control_time<<" (extimated bmp: "<<control_time*loaded_midi_tempo[curr_seg]<<")"<<endl;
	PortPerformance.SetMasterTempo(double(control_time));

}

void play ()
{
		
		//TEST starts here
/*		if (PortPerformance.IsPlaying(CSegment1)==S_OK)
		{
			PortPerformance.Stop(CSegment1);
		}
*/		
		SyncronizePlayback3();

		//TEST ends here
		//cout<<"set master tempo:"<<Time<<endl;
		//PortPerformance.SetMasterTempo(Time);		

}

bool filter (int Note)
{
	clock_t Now = clock() * CLK_TCK; 
	
	int elapTicks=0;
	
	if		(Note==NOTE_CRASH)   {elapTicks = Now - last_crash;   last_crash=Now;   dur_crash[0]=dur_crash[1];     dur_crash[1]=dur_crash[2];     dur_crash[2]=dur_crash[3];     dur_crash[3]=elapTicks;}
	else if (Note==NOTE_TOM1)    {elapTicks = Now - last_tom1;    last_tom1=Now;    dur_tom1[0]=dur_tom1[1];       dur_tom1[1]=dur_tom1[2];       dur_tom1[2]=dur_tom1[3];       dur_tom1[3]=elapTicks;}
	else if (Note==NOTE_TOM2)    {elapTicks = Now - last_tom2;    last_tom2=Now;    dur_tom2[0]=dur_tom2[1];       dur_tom2[1]=dur_tom2[2];       dur_tom2[2]=dur_tom2[3];       dur_tom2[3]=elapTicks;}
	else if (Note==NOTE_TOM3)    {elapTicks = Now - last_tom3;    last_tom3=Now;    dur_tom3[0]=dur_tom3[1];       dur_tom3[1]=dur_tom3[2];       dur_tom3[2]=dur_tom3[3];       dur_tom3[3]=elapTicks;}
	else if (Note==NOTE_HIHAT)   {elapTicks = Now - last_hihat;   last_hihat=Now;   dur_hihat[0]=dur_hihat[1];     dur_hihat[1]=dur_hihat[2];     dur_hihat[2]=dur_hihat[3];     dur_hihat[3]=elapTicks;}
	else if (Note==NOTE_KICK)    {elapTicks = Now - last_kick;    last_kick=Now;    dur_kick[0]=dur_kick[1];       dur_kick[1]=dur_kick[2];       dur_kick[2]=dur_kick[3];       dur_kick[3]=elapTicks;}
	else if (Note==NOTE_RIDE)    {elapTicks = Now - last_ride;    last_ride=Now;    dur_ride[0]=dur_ride[1];       dur_ride[1]=dur_ride[2];       dur_ride[2]=dur_ride[3];       dur_ride[3]=elapTicks;}
	else if (Note==NOTE_SNARE)   {elapTicks = Now - last_snare;   last_snare=Now;   dur_snare[0]=dur_snare[1];     dur_snare[1]=dur_snare[2];     dur_snare[2]=dur_snare[3];     dur_snare[3]=elapTicks;}
	else if (Note==NOTE_KICK_H)  {elapTicks = Now - last_kick_h;  last_kick_h=Now;  dur_kick_h[0]=dur_kick_h[1];   dur_kick_h[1]=dur_kick_h[2];   dur_kick_h[2]=dur_kick_h[3];   dur_kick_h[3]=elapTicks;}
	else if (Note==NOTE_KICK_H2) {elapTicks = Now - last_kick_h2; last_kick_h2=Now; dur_kick_h2[0]=dur_kick_h2[1]; dur_kick_h2[1]=dur_kick_h2[2]; dur_kick_h2[2]=dur_kick_h2[3]; dur_kick_h2[3]=elapTicks;}
	else
	{
		cout << "*** ERROR: UNKNOWN NOTE PROCESSED BY THE FILTER???? *** " << endl;
		elapTicks=0;
	}

	double elapMilli = elapTicks/1000;

	if (elapMilli<=150) 
	{	
		cout << "WARN: Double HIT!" << endl;
		if (hit_filter_enabled)
			return false;
		else return true;
	}
	else return true;
}

//****************************************************************

void note_received (unsigned char Command, unsigned char Channel, unsigned char Note, unsigned char Velocity)
{
		note_timer_s* timer_pointer;
				
 		if (sender)
		{
			sender->notify(static_cast<int> (Command),
							static_cast<int> (Channel),
							static_cast<int> (Note),
							static_cast<int> (Velocity));
			loopback = clock() * CLK_TCK; 
		}

		cout << " Received command " << static_cast<int> (Command) << static_cast<int>(Channel) << 
        " Note " << static_cast<int>(Note) << " with velocity " << static_cast<int>(Velocity) << endl;

		if (static_cast<int>(Velocity) >0)
		{
			if (!filter(Note)) return;

			OutPort.SendMidiMsg(COutputPort::EncodeMidiMsg(PATCH_CHANGE,8,8,0),0);

			// NOTES REMAPPING (for playback only)
			switch (static_cast<int>(Note))
			{
				case NOTE_SNARE:
				OutPort.SendMidiMsg(COutputPort::EncodeMidiMsg(NOTE_ON,8,SOUND_SNARE,127),0); 
				//COutPort.SendMidiMsg(COutputPort::EncodeMidiMsg(NOTE_ON,8,SOUND_HIHAT,127),0); 
				break;
				case NOTE_KICK:
				OutPort.SendMidiMsg(COutputPort::EncodeMidiMsg(NOTE_ON,8,SOUND_KICK,127),0); 
				//COutPort.SendMidiMsg(COutputPort::EncodeMidiMsg(NOTE_ON,8,SOUND_HIHAT,127),0); 
				break;
				case NOTE_KICK_H :
				OutPort.SendMidiMsg(COutputPort::EncodeMidiMsg(NOTE_ON,8,SOUND_KICK_H,127),0); 
				break;
				default:
				OutPort.SendMidiMsg(COutputPort::EncodeMidiMsg(NOTE_ON,8,static_cast<int>(Note),127),0); 
				break;
			}
			

			if (static_cast<int>(Note) == NOTE_KICK 
				|| static_cast<int>(Note) == NOTE_SNARE)
			{
				timer_pointer=&note_timer1;
				timer_pointer->Begin = timer_pointer->End; 
				timer_pointer->End = clock() * CLK_TCK; 
				//resets the watchdog that stops music after XXX seconds of no hits
				WatchBegin = clock() * CLK_TCK;

				if (timer_pointer->counter++ == 256) timer_pointer->counter = 1;				

				elapTicks = timer_pointer->End - timer_pointer->Begin;
				elapMilli = elapTicks/1000;
				bpm = 60000/ elapMilli;
				Hz = bpm/60;
				tTime = bpm/loaded_midi_tempo[curr_seg];

				cout << endl << "Count" << timer_pointer->counter<< endl;
				
				//***************************************
				//scoreGUIbpm = 60;
				//old_scoreGUIbpm = 60;
				//***************************************

				if (1)
				{
					if (start_required)
					{
						if (PortPerformance.IsPlaying(segment[curr_seg])==S_FALSE)
							{
							 if (playback_enabled==true )
								{
									PortPerformance.SetMasterTempo(double(scoreGUIbpm/loaded_midi_tempo[curr_seg]));
									PortPerformance.PlaySegment(segment[curr_seg]);
								}
							}
						start_required =false;
					}

					if (stop_required)
					{
						if (PortPerformance.IsPlaying(segment[curr_seg])==S_OK)
							{
							 if (playback_enabled==true )
								{
									PortPerformance.Stop(segment[curr_seg]);
								}
							}
						stop_required= false;
					}
				}

				if (0)
				//if (timer_pointer->counter%2 ==0)
				//if (timer_pointer->counter%4 ==0)
				{
					played_note.increase();
					//played_note.print();

					//check if the frequency given by the external gui matches with the one extimated here...
					if (!(bpm<scoreGUIbpm+10 &&
						  bpm>scoreGUIbpm-10))
					{
						cout << "*** WARN: scoreGUIbpm and bpm(extimated) differs a lot! ***" << endl;
					}
					else
					{
					play();	
					}
					//play();					
					if (scoreGUIbpm != old_scoreGUIbpm)
					{
						cout << "*** WARN: deteced a change of tempo received from the GUI! ***" << endl;
						play();	
					}
					
					/*
					just for debug
					{
						cout<<timer_pointer->counter;
						cout<<" elasped time:"<<elapMilli<< endl;
						cout<<"extimated bmp:"<<bpm;
						cout<<" (Hz: "<<Hz<<")";
						cout<<" (Time: "<<Time<<")"<<endl;
					}
					*/
				}
			}
		}
		else
		{
			OutPort.SendMidiMsg(COutputPort::EncodeMidiMsg(NOTE_OFF,8,static_cast<int>(Note),0),0); 
		}
}

// Overriden function for structured Midi data capture
void CDMReceiver::RecvMidiMsg(REFERENCE_TIME lprt,DWORD dwChannel,DWORD dwMsg)
{
	unsigned char Command,Channel,Note,Velocity;

	// Extract Midi parameters from a Midi message	
	CInputPort::DecodeMidiMsg(dwMsg,&Command,&Channel,&Note,&Velocity);
	
	if (Command == NOTE_ON) //Channel #0 Note-On
    {             
		note_received(Command,Channel,Note,Velocity);
	}
	else
	{
		if (static_cast<int> (Command) != 240)
		cout << " Received command " << static_cast<int> (Command) << "on channel " << static_cast<int>(Channel) << 
        " Note " << static_cast<int>(Note) << " with velocity " << static_cast<int>(Velocity) << endl;
	}
	
}


clock_t loopback_now;
int elaploop;

int main(int argc, char* argv[])
{
	CDirectMusic		CDMusic;
	CInputPort			CInPort;
	CDMReceiver			Receiver;

	CDLSLoader			CLoader;
	CCollection			CCollectionB;
	CInstrument			CInstrument1,CInstrument2;

	if (!initialized)
	{
		ppdebugger.open(0x378);
		initialized=true;
	}


	try
	{

		// Ititializes YARP Network
		Network::init ();
		sender=new Sender;
		sender->setRate(SENDER_RATE);
		sender->open("/midiDrum/server/out");
		sender->start();
		yarpPortIn.open("/midiDrum/server/in");
		yarpPortIn2.open("/midiDrum/server/freqin");
		
		// Initializes DirectMusic
		CDMusic.Initialize();

		// Initializes a port performance object
		PortPerformance.Initialize(CDMusic,NULL,NULL);

		// Initializes the Loader object	
		CLoader.Initialize();

		// Initializes ports given the DirectMusic manager object
		OutPort.Initialize(CDMusic);
		OutPort2.Initialize(CDMusic);
		CInPort.Initialize(CDMusic);

		// Info structure of the Midi port
		INFOPORT PortInfo, PortInfo2;
		DWORD dwPortCount = 0;

		//Software Synthesizer selection	
		dwPortCount = 2 ;
		do
			OutPort.GetPortInfo(++dwPortCount,&PortInfo);
		while (!(PortInfo.dwFlags & DMUS_PC_SOFTWARESYNTH));
		OutPort.SetPortParams(0,0,1,SET_REVERB | SET_CHORUS,44100);
		OutPort.ActivatePort(&PortInfo,32);
		dwPortCount = 4;
		do
			OutPort2.GetPortInfo(dwPortCount,&PortInfo2);
		while (!(PortInfo2.dwFlags & DMUS_PC_SOFTWARESYNTH));
		OutPort2.SetPortParams(1,1,1,SET_REVERB | SET_CHORUS,44100);
		OutPort2.ActivatePort(&PortInfo2,32);


		// Output port activation given the information of the port 
		cout << "\nSelected output port: " << PortInfo.szPortDescription << endl;
		cout << "\nSelected output port: " << PortInfo2.szPortDescription << endl;

		// Input port activation, select the first one (by default)
		CInPort.GetPortInfo(1,&PortInfo);
		CInPort.ActivatePort(&PortInfo,SYSTEM_EXCLUSIVE_MEM);
	
		// Sets up the receiver object
		CInPort.SetReceiver(Receiver);
		
		cout << "Selected input port: " << PortInfo.szPortDescription << endl;
		cout << "Press a key to load instruments..." << endl;
		
		getch();
		
		// Loads the deafault GM/GS collection of the software synthesizer
		CLoader.LoadDLS(NULL,CCollectionB);
		//	CLoader.LoadDLS(_T(".//media//sample.dls"),CCollectionB);		
		
		// Structure of the instrument information		
		INSTRUMENTINFO InstInfo;
		DWORD dwInstIndex = 0;
	
		// Enumerates instruments in CollectionB
		while (CCollectionB.EnumInstrument(dwInstIndex++,&InstInfo) == S_OK)
		{	
			cout << dwInstIndex   << " Instrument name: " << InstInfo.szInstName  << endl;
			cout << "Patch in collection: " << InstInfo.dwPatchInCollection << endl;
		}

		// Gets the instrument with index 214 from the CollectionB	
		//CCollectionB.GetInstrument(CInstrument1,231); //DRUM JAZZ
		//CCollectionB.GetInstrument(CInstrument1,232); //DRUM BRUSH
		CCollectionB.GetInstrument(CInstrument1,226); //DRUM STANDARD
		//CCollectionB.GetInstrument(CInstrument1,227); //DRUM ROOM
		//CCollectionB.GetInstrument(CInstrument1,1); //PIANO
		CCollectionB.GetInstrument(CInstrument2,214);

		// Assigns it to the Midi program 0	
		CInstrument2.SetPatch(0);
		CInstrument1.SetPatch(8);
		cout << "\nSelected instrument: " << CInstrument1.m_strInstName << endl;
		cout << "Source collection patch " << CInstrument1.m_dwPatchInCollection << 
			" to destination MIDI program: " << CInstrument1.m_dwPatchInMidi << endl;

		// Sets the note range	
		CInstrument1.SetNoteRange(0,127);
		CInstrument2.SetNoteRange(0,127);
		
		// Downloads the instruments to the output ports
		OutPort.DownloadInstrument(CInstrument1);
		OutPort.DownloadInstrument(CInstrument2);		
	//	OutPort2.DownloadInstrument(CInstrument1);
	//	OutPort2.DownloadInstrument(CInstrument2);	
		
		//Activates input Midi message handling 
		CInPort.ActivateNotification();
		cout << "Notification activated" << endl;
	
		// Redirects messages from source global channel 0 to destination global channel 0
		CInPort.SetThru(0,0,0,OutPort);
		cout << "Thru activated" << endl;
		cout << "Processing events..." << endl;	

		// **** WARNING: Don't close the application while receiving SysEx data 
		// this could hang the application since it is dumping the buffer into a 
		// for loop
	
		// Load the instruments in the MIDI programs
		OutPort.SendMidiMsg(COutputPort::EncodeMidiMsg(PATCH_CHANGE,0,0,0),0);
		OutPort.SendMidiMsg(COutputPort::EncodeMidiMsg(PATCH_CHANGE,8,8,0),0);
	
		cout << "Playing with the instrument1:" << CInstrument1.m_strInstName << endl;		
		cout << "Playing with the instrument2:" << CInstrument2.m_strInstName << endl;		
		OutPort.SendMidiMsg(COutputPort::EncodeMidiMsg(NOTE_ON,0,38,127),0); 
		OutPort.SendMidiMsg(COutputPort::EncodeMidiMsg(NOTE_ON,8,38,127),0);

		//------------------------	
		// Loads a MIDI file into the segment
		/*try	
		{
			CLoader.LoadSegment(_T("..\\media\\should.mid"),CSegment1,TRUE);
		}
		catch (CDMusicException& DMExcp)
		{
			cout << "unable to find '..\\media\\should.mid' file!" << endl ; 
			try	
			{
				CLoader.LoadSegment(_T(".\\media\\should.mid"),CSegment1,TRUE);
			}
			catch (CDMusicException& DMExcp)
			{
				cout << "unable to find '.\\media\\should.mid' file!" << endl ; 
			}
		}*/

		CLoader.LoadSegment(_T("C:\\software\\iCub\\src\\midiDrum\\media\\popcorn1.sgp"),segment[0],TRUE);
		CLoader.LoadSegment(_T("C:\\software\\iCub\\src\\midiDrum\\media\\should.mid"),segment[1],TRUE);
		CLoader.LoadSegment(_T("C:\\software\\iCub\\src\\midiDrum\\media\\Da_Da_Da.MID"),segment[2],TRUE);
		CLoader.LoadSegment(_T("C:\\software\\iCub\\src\\midiDrum\\media\\novemberrain.mid"),segment[3],TRUE);
		CLoader.LoadSegment(_T("C:\\software\\iCub\\src\\midiDrum\\media\\I.mid"),segment[4],TRUE);
		CLoader.LoadSegment(_T("C:\\software\\iCub\\src\\midiDrum\\media\\Moonlight shadow.sgp"),segment[5],TRUE);
		
		DMUS_TEMPO_PARAM def_tempo; 

		int i=0;
		
		for (i=0; i<max_seg; i++)
		{
			segment[i].GetDefaultTempo(&def_tempo);
			loaded_midi_tempo[i]=def_tempo.dblTempo;
			cout << "default tempo of the loaded MIDI track "<<i<<": " << loaded_midi_tempo[i] << endl;
		}
			
		//tempo inizialization
		pTimeSig.bBeatsPerMeasure=4;
		pTimeSig.bBeat=4;
		//pTimeSig.wGridsPerBeat=1;
		pTimeSig.wGridsPerBeat=8;
		pTimeSig.mtTime=0;

		//filter init
		for (i=0; i<4; i++)
		{
			dur_crash[i]=0;
			dur_tom1[i]=0;
			dur_tom2[i]=0;
			dur_tom3[i]=0;
			dur_hihat[i]=0;
			dur_kick[i]=0;
			dur_ride[i]=0;
			dur_snare[i]=0;
			dur_kick_h[i]=0;
			dur_kick_h2[i]=0;
		}

		WORD     len_measure = 0;
		BYTE     len_beat    = 0;
		BYTE     len_grid    = 0;
		short    len_offset  = 0;

		MUSIC_TIME def_length;
		segment[curr_seg].GetLength(&def_length);
		PortPerformance.TimeToRhythm(def_length,&pTimeSig,&len_measure,&len_beat,&len_grid,&len_offset);
		
		/*
		cout << "default length of the loaded MIDI track: ";
		cout << int(len_measure) << " ";
		cout << int(len_beat) << " ";
		cout << int(len_grid) << " ";
		cout << int(len_offset) << endl;
		*/

		last_measure = len_measure;
		played_note.set_last_measure(last_measure);
		
		// Repeats the segment until infinite
		for (i=0; i<max_seg; i++)
		{
			segment[i].SetRepeats(DMUS_SEG_REPEAT_INFINITE); 
		}

		// Adds the selected port to the performance
		PortPerformance.AddPort(OutPort2,0,1);

		// Downloads the segment to the performance
		for (i=0; i<max_seg; i++)
		{
			segment[i].Download(PortPerformance);
			segment[i].SetStartPoint(0);
		}

		// Plays the segment
		//PortPerformance.PlaySegment(CSegment1);
		//------------------------

		cout << "---------------------------------------------------------------------------" <<endl;
		cout << "yarp input port is '/midiDrum/server/freqin'" <<endl;
		cout << "yarp input port is '/midiDrum/server/in'" <<endl;
		cout << "yarp output port is '/midiDrum/server/out'" <<endl;
		cout << "" <<endl;
		cout << "list of commands on yarp input port:" <<endl;
		cout << "'1'-'6' selects the song" <<endl;
		cout << "'9' stops the song" <<endl;
		cout << "'0' quits the application" <<endl;
		cout << "'11' simulates a drum (on/off)" <<endl;
		cout << "'12' increases tempo of simulated drum" <<endl;
		cout << "'13' decreases tempo of simulated drum" <<endl;
		cout << "'14' sets hit filter (on/off)" <<endl;
		cout << "'15' sets song playback (on/off)" <<endl;
		cout << "" <<endl;
		cout << "*** hit the pads to test the MIDI communication ***" <<endl;
		cout << "*** press a key to enter the main loop ***" <<endl;
		cout << "---------------------------------------------------------------------------" <<endl;

		getch();
		OutPort.SendMidiMsg(COutputPort::EncodeMidiMsg(NOTE_OFF,0,40,0),0); 

		// TEST
		clock_t Begin = clock() * CLK_TCK ;
		clock_t End = clock() * CLK_TCK ;
		int flag1 =0;
		int flag2 =0;
		int flag3 =0;
		int flag_simulation = false;
		int c=255;
		Bottle *inputBottle = 0;
		Bottle *inputBottle2 = 0;

		do 
		{	
				//receives commands from the YarpPortIn and parses them
				inputBottle = yarpPortIn.read(false);
				inputBottle2 = yarpPortIn2.read(false);
				if (inputBottle2 != NULL)
				{
					/*
					if (inputBottle2->get(0).isInt())
						{
							c = inputBottle2->get(0).asInt();
							if (c==1)
							{
								loopback_now = clock() * CLK_TCK; 
								int elaploop = loopback_now - loopback;
								double elaploopmilli = elaploop/1000;
								cout <<  "loop " << elaploopmilli << endl;			
							}
						}
					*/
					if (inputBottle2->get(0).isDouble())
						{
							//received a change of tempo from drum manager
							double hz = inputBottle2->get(0).asDouble();
							
							//doubling the speed!
							hz = hz * 2;
							
							old_scoreGUIfreq = scoreGUIfreq;
							old_scoreGUIbpm  = scoreGUIbpm;							
							scoreGUIbpm = hz*60;
							scoreGUIfreq = scoreGUIbpm;	
							cout << "received freq of " << hz << " HZ from GUI ("<< scoreGUIbpm <<" bpm) ("<< loaded_midi_tempo[curr_seg] <<" orig bpm)" << endl;
							cout << "setting playback @ " << (double(scoreGUIbpm/loaded_midi_tempo[curr_seg])) << endl;
							PortPerformance.SetMasterTempo(double(scoreGUIbpm/loaded_midi_tempo[curr_seg]));
						}

				}
				if (inputBottle != NULL)
				{
					if (inputBottle->get(0).isInt())
						{
							c = inputBottle->get(0).asInt();


							switch (c) 
							{
								case 11:	
								// receiving a '1' char simulates a drum (on/off)
									flag_simulation=!flag_simulation;
								break;
								case 12:			
								// receiving a '2' char increases tempo of simulated drum							
									if (measure_lenght>=400)
									{
										measure_lenght-=10;
										cout << "increasing tempo, now is:" << measure_lenght;
										cout << " bpm is:" << 1/(measure_lenght/1000)*60;
										cout << endl;
									}
								break;
								case 13:		
								// receiving a '3' char decreases tempo of simulated drum
									if (measure_lenght>=400) 
									{
										measure_lenght+=10;
										cout << "decreasing tempo, now is:" << measure_lenght;
										cout << " bpm is:" << 1/(measure_lenght/1000)*60;
										cout << endl;
									}
								break;
								case 14:	
									// receiving a '4' char turns on/off the hit filter
									hit_filter_enabled=!hit_filter_enabled;
									if (hit_filter_enabled) cout << "hit filter ON" << endl;
									else  cout << "hit filter OFF" << endl;
								break;
								case 15:	
									// receiving a '5' char turns on/off the song playback
									playback_enabled=!playback_enabled;
									if (playback_enabled) cout << "enable song playback ON" << endl;
									else  cout << "enable song playback OFF" << endl;
								break;
								case 1:	
								case 2:
								case 3:
								case 4:
								case 5:
								case 6:
									curr_seg = c-1;
									cout << "selected song "<< curr_seg << " ,starting at the next beat..."<< endl;
									start_required=true;
								break;
								case 9:	
									stop_required=true;
									cout << "stopping song "<< curr_seg << endl;
								break;
							}

						}
				}
				if (stop_required)
				{
					if (PortPerformance.IsPlaying(segment[curr_seg])==S_OK)
						{
						 if (playback_enabled==true )
							{
								PortPerformance.Stop(segment[curr_seg]);
							}
						}
					stop_required= false;
				}

				End = clock() * CLK_TCK; 
				int elapTicks = End - Begin;
				double elapMilli = elapTicks/1000;
				
				//This is used to stop the playback if no drum hits are received for more than 3sec
				int watchTicks = End - WatchBegin;
				double watchMilli = watchTicks/1000;				
				if (PortPerformance.IsPlaying(segment[curr_seg])==S_OK)
				{
					if (watchMilli>6000)
					{
				//		cout << "*** NO MIDI notes received for more than 3seconds, stopping the playback..." << endl;
				//		PortPerformance.Stop(segment[curr_seg]);
					}
				}

				//This is used to simulate MIDI drum events
				if (flag_simulation)
					{
					if (elapMilli >= measure_lenght/4.0-m_eta && elapMilli<= measure_lenght/4.0+m_eta && flag2==0)
					{
						flag2=1;
						note_received (NOTE_ON, 0, NOTE_CRASH, 127);
					}		
					if (elapMilli >= measure_lenght/2.0-m_eta && elapMilli<= measure_lenght/2.0+m_eta && flag1==0)
					{
						flag1=1;			
						note_received (NOTE_ON, 0, NOTE_SNARE, 127);
					}
					if (elapMilli >= measure_lenght*3.0/4.0-m_eta && elapMilli<= measure_lenght*3.0/4.0+m_eta && flag3==0)
					{
						flag3=1;
						note_received (NOTE_ON, 0, NOTE_HIHAT, 127);
					}	
					if (elapMilli >= measure_lenght)
					{
						Begin =End; 
						note_received (NOTE_ON, 0, NOTE_KICK, 127);
						flag1=0;
						flag2=0;
						flag3=0;
					}
				}
				//Time::delay(0.3);
		}
		while (c!=0);

		//release segment and performance
		for (i=0; i<max_seg; i++)
		{
			segment[i].Unload(PortPerformance);
			segment[i].ReleaseSegment();
		}
		PortPerformance.ReleasePerformance();

		// Ends the application
		// Breaks the redirection
		CInPort.BreakThru(0,0,0);

		// Ends the notification
		CInPort.TerminateNotification();
		cout << "\n\nNotification finished" << endl;

		// Unload the segment
		//CSegment1.UnloadAllPerformances();

		cout << "Unloading collections" << endl; 
		// Unloads the collections from the loader
		CLoader.UnloadCollection(CCollectionB);
	
		// Unloads the instruments from the port
		cout << "Unloading instruments" << endl;
		OutPort.UnloadInstrument(CInstrument1);
		OutPort.UnloadInstrument(CInstrument2);
		OutPort.ReleasePort();

		// Closes YARP Network
		sender->stop();
		sender->close();
		delete sender;
		Network::fini ();

		cout << "Application finished...OK" << endl;
		
		getch();

	} 
	catch (CDMusicException& DMExcp)
	{
		cout << "\n" << DMExcp.GetErrorDescription() << "\n" << endl;	
	}
	return 0;
}
