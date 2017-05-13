#include "selectioncheckbox.h"
#include <QtConcurrent/QtConcurrent>

SelectionCheckBox::SelectionCheckBox(QWidget *parent) : QCheckBox(parent)
{

}

SelectionCheckBox::SelectionCheckBox(FirmwareUpdaterCore *core, QTreeWidgetItem *it, QWidget *parent) : QCheckBox(parent)
{
    this->selected = false;
    //this->core = core;
//    isEth = false;
//    isCan = false;
//    treeNode = it;



//    if(treeNode->data(0,DEVICE_LEVEL).toInt() == 2){
//        if(treeNode->parent()->text(DEVICE) == "ETH"){
//            isEth = true;
//            isCan = false;
//        }else{
//            isEth = false;
//            isCan = true;
//        }
//    }else if(treeNode->data(0,DEVICE_LEVEL).toInt() == 3){
//        isEth = false;
//        isCan = true;
//    }

    connect(this,SIGNAL(toggled(bool)),this,SLOT(onSelectionChanged(bool)));

}


//bool SelectionCheckBox::boardIsEth()
//{
//    return isEth && !isCan;
//}

//bool SelectionCheckBox::boardIsCan()
//{
//    return isCan && !isEth;
//}


void SelectionCheckBox::onSelectionChanged(bool selected)
{
    setSelected(selected);
}

void SelectionCheckBox::setSelected(bool selected)
{
    //needLoading(true,selected);
    this->selected = selected;
    needChangeSelection(selected);



//    if(isEth){
//        core->setSelectedEthBoard(getBoardIndex(),selected);
//    }else{
//        QString ethAddress;
//        if(treeNode->data(0,DEVICE_LEVEL).toInt() == 3){
//            ethAddress = treeNode->parent()->text(ADDRESS);
//        }
//        QtConcurrent::run(core,&FirmwareUpdaterCore::setSelectedCanBoard,getBoardIndex(),selected,ethAddress);
//        //core->setSelectedCanBoard(getBoardIndex(),selected,ethAddress);
//    }

}

void SelectionCheckBox::onSelectEnded()
{
    //needLoading(false,false);
    selectedChanged(selected);
}

bool SelectionCheckBox::isSelected()
{
    return selected;
}



//QTreeWidgetItem *SelectionCheckBox::getTreeNode()
//{
//    return treeNode;
//}


//QString SelectionCheckBox::getBoardType()
//{
//    return treeNode->text(DEVICE);
//}

//int SelectionCheckBox::getBoardIndex()
//{
//    return treeNode->data(0,INDEX_OF_BOARD).toInt();
//}
