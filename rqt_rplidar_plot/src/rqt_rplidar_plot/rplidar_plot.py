from rqt_gui_py.plugin import Plugin
from .rplidar_plot_widget import RPLidarPlotWidget

class RPLidarPlotPlugin(Plugin):
    def __init__(self, context):
        super(RPLidarPlotPlugin, self).__init__(context)
        self.setObjectName('RPLidarPlotPlugin')

        # Create the main widget
        self._widget = RPLidarPlotWidget()

        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        # Add widget to the user interface
        context.add_widget(self._widget)

    def shutdown_plugin(self):
        # This method is called when the plugin is closed
        self._widget.shutdown()

    # The following methods are not used in this simple plugin,
    # but are good practice to include.
    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass

    # def trigger_configuration(self):
    #     # Commented out as there's no configuration dialog for this plugin
    #     pass