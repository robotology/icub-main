/****************************************************************************
 ** ui.h extension file, included from the uic-generated form implementation.
 **
 ** If you want to add, delete, or rename functions or slots, use
 ** Qt Designer to update this file, preserving your code.
 **
 ** You should not define a constructor or destructor in this file.
 ** Instead, write your code in functions called init() and destroy().
 ** These will automatically be called by the form's constructor and
 ** destructor.
 *****************************************************************************/

#include <iostream>
#include <fstream>
#include <sstream>
#include <qdir.h>
#include <cstdlib>

using namespace std;

void mainDlg::init()
{
    nbparts =5;
    string parts[5] = {"left_arm","right_arm","left_leg","right_leg","head"};
    
    ScoreSize=16;
    ///init variables
    this->currentTempo = 0.5; //in Hz
    for(int i=0;i<nbparts;i++)
        this->currentPhase[i] = 0.0;
         
    //we open commnunication ports
    bool ok= Network::checkNetwork();
    if(!ok)
    {
	exit(-1);
    }
    Network::init();
    
    interactivePort.open("/interactive/out");
    Network::connect("/interactive/out","/interactive/in");
    
    for(int i=0;i<nbparts;i++)
        {
            string tmp_in = "/";
            tmp_in += parts[i];
            tmp_in += "/score/in";
            string tmp_out = "/";
            tmp_out += parts[i];
            tmp_out += "/score/out";
            partitionsPort[i].open(tmp_out.c_str());
            bool ok = Network::connect(tmp_out.c_str(),tmp_in.c_str());
            if(!ok)
                {
                    cout << "limb " << parts[i].c_str() << " not used" << endl;
                    usedPart[i] = false;
                    switch(i)
                        {
                        case 0:
                            leftArmBox->setEnabled(false);
                            break;
                        case 1:
                            rightArmBox->setEnabled(false);
                            break;
                        case 2:
                            leftLegBox->setEnabled(false);
                            break;
                        case 3:
                            rightLegBox->setEnabled(false);
                            break;
                        case 4:
                            headBox->setEnabled(false);
                            break;
                        }
                }
            else
                {
                    usedPart[i]=true;
                    string tmp_in = "/";	
                    tmp_in += parts[i];
                    tmp_in += "/phase/in";
                    string tmp_out = "/";
                    tmp_out += parts[i];
                    tmp_out += "/phase/out";
                    phasePort[i].open(tmp_out.c_str());
                    bool ok2= Network::connect(tmp_out.c_str(),tmp_in.c_str());
                    if(!ok2){cout << "Problems connecting to phase port port for part" << parts[i].c_str() << endl;}
                }
        }
        
    ///we read the partition folder
    updatePartitionFolder();
    
    //we reset the custom stuff and the partition
    for(int i=0;i<2*nbparts+1;i++)
        {
            double pini[11]={0.0,0.0,0.0,0.0,0.0,0.5,0.0,0.0,0.0,0.0,0.0};
            currentPartition[i].clear();
            for(int j=0;j<ScoreSize;j++)
                currentPartition[i].push_back(pini[i]);	
        }
    
    for(int i=0; i<nbparts;i++)
        {
            customBeat[i]=0;
        }
}


void mainDlg::freqSlider_valueChanged( int val)
{
    currentTempo = ((double)val)/100.0;
    
    QString newval;
    QTextOStream(&newval) << "Tempo " << currentTempo << " Hz";
    this->freqLabel->setText(newval);
}


void mainDlg::paritionPushButton_clicked()
{
    QString partName = this->partitionListBox->currentText();
    if(!partName.isNull())
        {
            ifstream myFile;
            string name = "part/";
            name += partName.latin1();
            myFile.open(name.c_str());
            if(!myFile.is_open())
                {
                    cout << "cannot read score " << name << endl;
                    return;
                }
            cout << "playing score: " << name.c_str() << endl;
	
            for(int i=0;i<2*nbparts+1;i++)
                {
                    char line[255];
                    myFile.getline(line,255);
                    string myString(line);
                    stringstream linestream(myString);
                    currentPartition[i].clear();
                    string temp;
                    linestream >> temp;
                    while(!linestream.eof())
                        {
                            double p;
                            linestream >> p;
                            currentPartition[i].push_back(p);
                        }
                }
	
            for(int i=0;i<2*nbparts+1;i++)
                {
                    cout << "score for " << i << " is ";
                    for(int j=0;j<currentPartition[i].size();j++)
                        cout << currentPartition[i][j] << " ";
                    cout << endl;
                }
            sendPartitions();
        }
}


void mainDlg::updatePartitionFolder()
{
    ///we read the partition folder
    QDir myDir("part/");
    QStringList files = myDir.entryList("*.csv",QDir::Files);
    QStringList::Iterator it = files.begin();
    while(it!=files.end())
        {	
            cout << (*it).ascii() << endl;
            this->partitionListBox->insertItem(*it);
            it++;
        }
}


void mainDlg::freqSlider_sliderReleased()
{
    //the slider was released, we send the new parameters to the interface
    sendParameters();
}


void mainDlg::partitionStopPushButton_clicked()
{
    stopPartition();
    sendPartitions();
}


void mainDlg::sendPartitions()
{
    for(int i =0;i<nbparts;i++)
        {
            if(usedPart[i])
                {    
                    Bottle &bot = partitionsPort[i].prepare();
                    bot.clear();
                    cout << "sending score part " << i << ":   ";
                    for(int k=0;k<ScoreSize;k++)
                        {
                            bot.addInt((int)currentPartition[i][k]);
                            cout << (int)currentPartition[i][k] << " ";
                        }
                    cout << endl;
                    partitionsPort[i].write();
	    
                    Bottle &bot2 = phasePort[i].prepare();
                    bot2.clear();
		    cout << "sending phase shift information for part " << i <<":    ";
                    for(int k=0; k<ScoreSize;k++)
                        {
                            bot2.addDouble(currentPartition[i+6][k]);
                            cout << currentPartition[i+6][k] << " ";	    
                        }
                    cout << endl;
                    phasePort[i].write();
                }	
        }	
    
    Bottle& bot = interactivePort.prepare();
    bot.clear();
    cout << "sending frequency:  ";
    for(int k=0; k<ScoreSize;k++)
        {
            bot.addDouble(currentPartition[5][k]);
            cout << currentPartition[5][k] << " ";	    
        }
    cout << endl;
    interactivePort.write();
}


void mainDlg::stopPartition()
{
    double pfin[11]={0.0,0.0,0.0,0.0,0.0,0.5,0.0,0.0,0.0,0.0,0.0};
    for(int i=0;i<nbparts;i++)
        {	
        currentPartition[i].clear();
        for(int j=0;j<ScoreSize;j++)
            currentPartition[i].push_back(pfin[i]);
        }        
}


void mainDlg::customStopPushButton_clicked()
{
    //stopPartition();
    //sendPartitions();
    QColor white(255,255,255);
    QColor red(255,0,0);
    
    larmButton1->setPaletteBackgroundColor(white);
    larmButton2->setPaletteBackgroundColor(white);
    larmButton3->setPaletteBackgroundColor(white);
    larmButton4->setPaletteBackgroundColor(red);
    
    rarmButton1->setPaletteBackgroundColor(white);
    rarmButton2->setPaletteBackgroundColor(white);
    rarmButton3->setPaletteBackgroundColor(white);
    rarmButton4->setPaletteBackgroundColor(red);
    
    llegButton1->setPaletteBackgroundColor(white);
    llegButton4->setPaletteBackgroundColor(red);
    
    rlegButton1->setPaletteBackgroundColor(white);
    rlegButton4->setPaletteBackgroundColor(red);
    
    headButton1->setPaletteBackgroundColor(white);
    headButton2->setPaletteBackgroundColor(white);
    headButton3->setPaletteBackgroundColor(white);
    headButton4->setPaletteBackgroundColor(white);
    headButton5->setPaletteBackgroundColor(red);
    headButton6->setPaletteBackgroundColor(white);
    
    for(int i=0;i<nbparts;i++)
    {
	customBeat[i] = 0;
	generate_and_playCustom(i);
    }	
    
    
}


void mainDlg::sendParameters()
{
    double phase[5];
   
    phase[4]=0.0;
    phase[3] = 0.0;
    phase[2] = currentPhase[1];
    phase[1] = currentPhase[2];
    phase[0] = phase[1] + currentPhase[0];
    
    Bottle& bot = interactivePort.prepare();
    bot.clear();
    for(int i=0; i<ScoreSize;i++)
    {
	bot.addDouble(currentTempo);
	currentPartition[5][i]=currentTempo;
    }
    interactivePort.write();
    
    for(int k=0; k<nbparts; k++)
        {
            Bottle& bot = phasePort[k].prepare();
            bot.clear();
            for(int i=0; i<ScoreSize;i++)
	    {
		bot.addDouble(phase[k]);
		currentPartition[k+6][i]=phase[k];
	    }
            phasePort[k].write();	
        }
}


void mainDlg::armsSlider_valueChanged( int val)
{
    double value = ((double)val);
    value = (value - 50.0)/50.0 * 3.14;
    currentPhase[0] = (double)((int)(value*100.0))/100.0;
    
    QString newval;
    QTextOStream(&newval) << "Arms  " << currentPhase[0];
    this->armsLabel->setText(newval);
}


void mainDlg::armsSlider_sliderReleased()
{
    sendParameters();
}


void mainDlg::legsSlider_valueChanged( int val)
{
    double value = ((double)val);
    value = (value - 50.0)/50.0 * 3.14;
    currentPhase[1] = (double)((int)(value*100.0))/100.0;
    
    QString newval;
    QTextOStream(&newval) << "Legs " << currentPhase[1];
    this->legsLabel->setText(newval);
}


void mainDlg::legsSlider_sliderReleased()
{
    sendParameters();
}


void mainDlg::armlegSlider_valueChanged( int val)
{
    double value = ((double)val);
    value = (value - 50.0)/50.0 * 3.14;
    currentPhase[2] = (double)((int)(value*100.0))/100.0;
    
    QString newval;
    QTextOStream(&newval) << "Arm-Leg " << currentPhase[2];
    this->armlegLabel->setText(newval);
}


void mainDlg::armlegSlider_sliderReleased()
{
    sendParameters();
}


void mainDlg::generate_and_playCustom(int i)
{
    currentPartition[i].clear();	
    for(int j=0;j<ScoreSize;j++)
	currentPartition[i].push_back(customBeat[i]);
    sendPartitions();
}

void mainDlg::larmButton1_clicked()
{
    QColor white(255,255,255);
    QColor red(255,0,0);
    larmButton1->setPaletteBackgroundColor(red);
    larmButton2->setPaletteBackgroundColor(white);
    larmButton3->setPaletteBackgroundColor(white);
    larmButton4->setPaletteBackgroundColor(white);
    customBeat[0] = 1;
    generate_and_playCustom(0);
}


void mainDlg::larmButton2_clicked()
{
    QColor white(255,255,255);
    QColor red(255,0,0);
    larmButton1->setPaletteBackgroundColor(white);
    larmButton2->setPaletteBackgroundColor(red);
    larmButton3->setPaletteBackgroundColor(white);
    larmButton4->setPaletteBackgroundColor(white);
    customBeat[0] = 2;
    generate_and_playCustom(0);
}


void mainDlg::larmButton3_clicked()
{
    QColor white(255,255,255);
    QColor red(255,0,0);
    larmButton1->setPaletteBackgroundColor(white);
    larmButton2->setPaletteBackgroundColor(white);
    larmButton3->setPaletteBackgroundColor(red);
    larmButton4->setPaletteBackgroundColor(white);
    customBeat[0] = 3;
    generate_and_playCustom(0);
}


void mainDlg::larmButton4_clicked()
{
    QColor white(255,255,255);
    QColor red(255,0,0);
    larmButton1->setPaletteBackgroundColor(white);
    larmButton2->setPaletteBackgroundColor(white);
    larmButton3->setPaletteBackgroundColor(white);
    larmButton4->setPaletteBackgroundColor(red);
    customBeat[0] = 0;
    generate_and_playCustom(0);
}

void mainDlg::rarmButton1_clicked()
{
    QColor white(255,255,255);
    QColor red(255,0,0);
    rarmButton1->setPaletteBackgroundColor(red);
    rarmButton2->setPaletteBackgroundColor(white);
    rarmButton3->setPaletteBackgroundColor(white);
    rarmButton4->setPaletteBackgroundColor(white);
    customBeat[1] = 1;
    generate_and_playCustom(1);
}


void mainDlg::rarmButton2_clicked()
{
    QColor white(255,255,255);
    QColor red(255,0,0);
    rarmButton1->setPaletteBackgroundColor(white);
    rarmButton2->setPaletteBackgroundColor(red);
    rarmButton3->setPaletteBackgroundColor(white);
    rarmButton4->setPaletteBackgroundColor(white);
    customBeat[1] = 2;
    generate_and_playCustom(1);
}


void mainDlg::rarmButton3_clicked()
{
    QColor white(255,255,255);
    QColor red(255,0,0);
    rarmButton1->setPaletteBackgroundColor(white);
    rarmButton2->setPaletteBackgroundColor(white);
    rarmButton3->setPaletteBackgroundColor(red);
    rarmButton4->setPaletteBackgroundColor(white);
    customBeat[1] = 3;
    generate_and_playCustom(1);
}


void mainDlg::rarmButton4_clicked()
{
    QColor white(255,255,255);
    QColor red(255,0,0);
    rarmButton1->setPaletteBackgroundColor(white);
    rarmButton2->setPaletteBackgroundColor(white);
    rarmButton3->setPaletteBackgroundColor(white);
    rarmButton4->setPaletteBackgroundColor(red);
    customBeat[1] = 0;
    generate_and_playCustom(1);
}

void mainDlg::headButton1_clicked()
{
    QColor white(255,255,255);
    QColor red(255,0,0);
    headButton1->setPaletteBackgroundColor(red);
    headButton2->setPaletteBackgroundColor(white);
    headButton3->setPaletteBackgroundColor(white);
    headButton4->setPaletteBackgroundColor(white);
    headButton5->setPaletteBackgroundColor(white);
    headButton6->setPaletteBackgroundColor(white);
    customBeat[4] = 1;
    generate_and_playCustom(4);
}


void mainDlg::headButton2_clicked()
{
    QColor white(255,255,255);
    QColor red(255,0,0);
    headButton1->setPaletteBackgroundColor(white);
    headButton2->setPaletteBackgroundColor(red);
    headButton3->setPaletteBackgroundColor(white);
    headButton4->setPaletteBackgroundColor(white);
    headButton5->setPaletteBackgroundColor(white);
    headButton6->setPaletteBackgroundColor(white);
    customBeat[4] = 2;
    generate_and_playCustom(4);
}


void mainDlg::headButton3_clicked()
{
    QColor white(255,255,255);
    QColor red(255,0,0);
    headButton1->setPaletteBackgroundColor(white);
    headButton2->setPaletteBackgroundColor(white);
    headButton3->setPaletteBackgroundColor(red);
    headButton4->setPaletteBackgroundColor(white);
    headButton5->setPaletteBackgroundColor(white);
    headButton6->setPaletteBackgroundColor(white);
    customBeat[4] = 3;
    generate_and_playCustom(4);
}

void mainDlg::headButton4_clicked()
{
    QColor white(255,255,255);
    QColor red(255,0,0);
    headButton1->setPaletteBackgroundColor(white);
    headButton2->setPaletteBackgroundColor(white);
    headButton3->setPaletteBackgroundColor(white);
    headButton4->setPaletteBackgroundColor(red);
    headButton5->setPaletteBackgroundColor(white);
    headButton6->setPaletteBackgroundColor(white);
    customBeat[4] = 4;
    generate_and_playCustom(4);
}


void mainDlg::headButton5_clicked()
{
    QColor white(255,255,255);
    QColor red(255,0,0);
    headButton1->setPaletteBackgroundColor(white);
    headButton2->setPaletteBackgroundColor(white);
    headButton3->setPaletteBackgroundColor(white);
    headButton4->setPaletteBackgroundColor(white);
    headButton5->setPaletteBackgroundColor(red);
    headButton6->setPaletteBackgroundColor(white);
    customBeat[4] = 0;
    generate_and_playCustom(4);
}

void mainDlg::headButton6_clicked()
{
    QColor white(255,255,255);
    QColor red(255,0,0);
    QColor blue(0,0,255);
    headButton1->setPaletteBackgroundColor(white);
    headButton2->setPaletteBackgroundColor(white);
    headButton3->setPaletteBackgroundColor(white);
    headButton4->setPaletteBackgroundColor(white);
    headButton5->setPaletteBackgroundColor(red);
    headButton6->setPaletteBackgroundColor(blue);
    customBeat[4] = 5;
    generate_and_playCustom(4);
}

void mainDlg::llegButton1_clicked()
{
    QColor white(255,255,255);
    QColor red(255,0,0);
    llegButton1->setPaletteBackgroundColor(red);
    llegButton4->setPaletteBackgroundColor(white);
    customBeat[2] = 1;
    generate_and_playCustom(2);
}

void mainDlg::llegButton4_clicked()
{
    QColor white(255,255,255);
    QColor red(255,0,0);
    llegButton1->setPaletteBackgroundColor(white);
    llegButton4->setPaletteBackgroundColor(red);
    customBeat[2] = 0;
    generate_and_playCustom(2);
}


void mainDlg::rlegButton1_clicked()
{
    QColor white(255,255,255);
    QColor red(255,0,0);
    rlegButton1->setPaletteBackgroundColor(red);
    rlegButton4->setPaletteBackgroundColor(white);
    customBeat[3] = 1;
    generate_and_playCustom(3);
}

void mainDlg::rlegButton4_clicked()
{
    QColor white(255,255,255);
    QColor red(255,0,0);
    rlegButton1->setPaletteBackgroundColor(white);
    rlegButton4->setPaletteBackgroundColor(red);
    customBeat[3] = 0;
    generate_and_playCustom(3);
}


void mainDlg::closeButton_clicked()
{ 
    QColor red(255,0,0);
    closeButton->setPaletteBackgroundColor(red);
    Bottle& bot = interactivePort.prepare();
    bot.clear();
    for(int i=0; i<ScoreSize;i++)
	bot.addDouble(-1.0);	
    interactivePort.write(); 
    
    Time::delay(0.5);
    interactivePort.close();
    for(int i=0;i<nbparts;i++)
    {
	if(usedPart[i])
	{
	    partitionsPort[i].close();
	    phasePort[i].close();
	}
    }
    
    exit(-1);
}

void mainDlg::customPlayPushButton_clicked()
{
    for(int i=0;i<nbparts;i++)
    {
	generate_and_playCustom(i);
    }
}

