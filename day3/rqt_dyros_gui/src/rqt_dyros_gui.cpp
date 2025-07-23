#include "rqt_dyros_gui/rqt_dyros_gui.h"
#include <pluginlib/class_list_macros.hpp>
#include <QStringList>

namespace rqt_dyros_gui{
    RqtDyrosPlugin::RqtDyrosPlugin() : rqt_gui_cpp::Plugin(), widget_(0){
        // Constructor is called first before initPlugin function, needless to say.

        //give QObjects reasonable names
        setObjectName("RqtDyrosPlugin");
    }

    void RqtDyrosPlugin::initPlugin(qt_gui_cpp::PluginContext &context){
        // access standalone command line arguments
        QStringList argv = context.argv();
        // create QWidget
        widget_ = new QWidget();
        // extend the widget with all attributes and children from UI file
        ui_.setupUi(widget_);
        // add widget to the user interface
        context.addWidget(widget_);

        myPub = node_->create_publisher<std_msgs::msg::Float32>("pub", 100);
        mySub = node_->create_subscription<std_msgs::msg::Float32>("sub", 100, std::bind(&RqtDyrosPlugin::subCallback, this, std::placeholders::_1));

        connect(ui_.myPushButton, SIGNAL(clicked()), this, SLOT(pushbutton()));
    }

    void RqtDyrosPlugin::pushbutton(){
        std::cout << "Hello World" << std::endl;

        ui_.myLabel->setText(QString::fromUtf8("Hello World"));

        auto message = std_msgs::msg::Float32();
        message.data = ui_.lineEdit->text().toFloat();

        myPub->publish(message);
    }

    void RqtDyrosPlugin::subCallback(const std_msgs::msg::Float32::SharedPtr msg) const{
        ui_.myLabel->setText(QString::number(msg->data));
    }

    void RqtDyrosPlugin::shutdownPlugin(){
        // TODO unregister all publishers here
    }

    void RqtDyrosPlugin::saveSettings(qt_gui_cpp::Settings &plugin_settings, qt_gui_cpp::Settings &instance_settings) const{
        (void)plugin_settings;
        (void)instance_settings;
        // TODO save intrinsic configuration, usually using:
        // instance_settings.setValue(k, v)
    }

    void RqtDyrosPlugin::restoreSettings(const qt_gui_cpp::Settings &plugin_settings, const qt_gui_cpp::Settings &instance_settings){
        (void)plugin_settings;
        (void)instance_settings;
        // TODO restore intrinsic configuration, usually using:
        // v = instance_settings.value(k)
    }
}

PLUGINLIB_EXPORT_CLASS(rqt_dyros_gui::RqtDyrosPlugin, rqt_gui_cpp::Plugin)