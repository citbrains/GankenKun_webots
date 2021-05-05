/********************************************************************************
** Form generated from reading UI file 'HCDelegate.ui'
**
** Created by: Qt User Interface Compiler version 4.8.7
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_HCDELEGATE_H
#define UI_HCDELEGATE_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLineEdit>
#include <QtGui/QMainWindow>
#include <QtGui/QPushButton>
#include <QtGui/QRadioButton>
#include <QtGui/QStatusBar>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_HCDelegateMainWindow
{
public:
    QAction *actionOpen;
    QWidget *centralwidget;
    QVBoxLayout *verticalLayout;
    QHBoxLayout *horizontalLayout_2;
    QRadioButton *radioSerialPort;
    QRadioButton *radioTCP;
    QHBoxLayout *horizontalLayout;
    QLineEdit *linePort;
    QPushButton *pushConnect;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *HCDelegateMainWindow)
    {
        if (HCDelegateMainWindow->objectName().isEmpty())
            HCDelegateMainWindow->setObjectName(QString::fromUtf8("HCDelegateMainWindow"));
        HCDelegateMainWindow->resize(346, 90);
        actionOpen = new QAction(HCDelegateMainWindow);
        actionOpen->setObjectName(QString::fromUtf8("actionOpen"));
        centralwidget = new QWidget(HCDelegateMainWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        verticalLayout = new QVBoxLayout(centralwidget);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        radioSerialPort = new QRadioButton(centralwidget);
        radioSerialPort->setObjectName(QString::fromUtf8("radioSerialPort"));

        horizontalLayout_2->addWidget(radioSerialPort);

        radioTCP = new QRadioButton(centralwidget);
        radioTCP->setObjectName(QString::fromUtf8("radioTCP"));
        radioTCP->setChecked(true);

        horizontalLayout_2->addWidget(radioTCP);


        verticalLayout->addLayout(horizontalLayout_2);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        linePort = new QLineEdit(centralwidget);
        linePort->setObjectName(QString::fromUtf8("linePort"));

        horizontalLayout->addWidget(linePort);

        pushConnect = new QPushButton(centralwidget);
        pushConnect->setObjectName(QString::fromUtf8("pushConnect"));

        horizontalLayout->addWidget(pushConnect);


        verticalLayout->addLayout(horizontalLayout);

        HCDelegateMainWindow->setCentralWidget(centralwidget);
        statusbar = new QStatusBar(HCDelegateMainWindow);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        HCDelegateMainWindow->setStatusBar(statusbar);

        retranslateUi(HCDelegateMainWindow);

        QMetaObject::connectSlotsByName(HCDelegateMainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *HCDelegateMainWindow)
    {
        HCDelegateMainWindow->setWindowTitle(QApplication::translate("HCDelegateMainWindow", "HCDelegate", 0, QApplication::UnicodeUTF8));
        actionOpen->setText(QApplication::translate("HCDelegateMainWindow", "Open", 0, QApplication::UnicodeUTF8));
        radioSerialPort->setText(QApplication::translate("HCDelegateMainWindow", "Serial Port", 0, QApplication::UnicodeUTF8));
        radioTCP->setText(QApplication::translate("HCDelegateMainWindow", "TCP", 0, QApplication::UnicodeUTF8));
        linePort->setInputMask(QString());
        linePort->setText(QApplication::translate("HCDelegateMainWindow", "127.0.0.1", 0, QApplication::UnicodeUTF8));
        pushConnect->setText(QApplication::translate("HCDelegateMainWindow", "Connect", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class HCDelegateMainWindow: public Ui_HCDelegateMainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_HCDELEGATE_H
