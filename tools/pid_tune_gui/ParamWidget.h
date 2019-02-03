//
//
//
//
//

#ifndef PARAMWIDGET_H_
#define PARAMWIDGET_H_

    #include <QHBoxLayout>
    #include <QPushButton>
    #include <QLineEdit>
    #include <QGroupBox>
    #include <fastcom/fastcom.h>

    class ParamWidget : public QHBoxLayout {
        Q_OBJECT
    public:
        ParamWidget(std::string _name, float _default = 0.0f);
        virtual ~ParamWidget(){};
    
        void setValue(float _value);
    private slots:
        void sendCallback();
    private:
        QLineEdit   *mEntry, *mParamName;
        QPushButton *mButton;
        std::string mServiceTopic;
        
    };
    
#endif