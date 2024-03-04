#include "handeyecalibratedialog.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    QApplication::setOrganizationDomain("Winerva");
    QApplication::setOrganizationDomain("winerva.com");
    QApplication::setApplicationName("WinervaHandEye");

    HandEyeCalibrateDialog dialog;
    dialog.show();

    return a.exec();
}
