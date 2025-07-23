#ifndef RQT_DYROS_H
#define RQT_DYROS_H

#include <rqt_gui_cpp/rqt_gui_cpp/plugin.h>
#include <ui_rqt_dyros_gui.h>
#include <QWidget>
#include <std_msgs/msg/float32.hpp>
#include <iostream>

namespace rqt_dyros_gui{
    class RqtDyrosPlugin : public rqt_gui_cpp::Plugin{
        Q_OBJECT
        public:
            RqtDyrosPlugin();
            virtual void initPlugin(qt_gui_cpp::PluginContext& context);
            virtual void shutdownPlugin();
            virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
            virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

            void subCallback(const std_msgs::msg::Float32::SharedPtr msg) const;

        protected slots:
            virtual void pushbutton();

        private:
            rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr myPub;
            rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr mySub;

            Ui::RqtDyrosGuiWidget ui_;
            QWidget* widget_;
    };
}

#endif