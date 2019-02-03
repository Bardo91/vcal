//
//
//
//
//


#include "ParamWidget.h"

ParamWidget::ParamWidget(std::string _name, float _default){
    mParamName = new QLineEdit(QString(_name.c_str()));
    mParamName->setEnabled(false);
    mParamName->setMaximumWidth(40);
    this->addWidget(mParamName);
    
    //mEntry = new QLineEdit(QString(std::to_string(_default).c_str()));
    mEntry = new QLineEdit();
    mEntry->setText(QString::number(_default,'g',3));
    mEntry->setMaximumWidth(50);
    //mEntry->setInputMask("0.000");
    this->addWidget(mEntry);
    
    // mButton = new QPushButton((_name).c_str());
    // mButton->setMaximumWidth(40);
    // this->addWidget(mButton);

    // connect(mButton, SIGNAL (released()), this, SLOT (sendCallback()));

}

void ParamWidget::setValue(float _value){
    mEntry->setText(QString::number(_value,'g',3));
}

void ParamWidget::sendCallback(){
    
}
