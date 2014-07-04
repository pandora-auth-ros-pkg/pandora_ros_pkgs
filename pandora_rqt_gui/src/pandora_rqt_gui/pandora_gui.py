from qt_gui.plugin import Plugin
from .main_widget import MainWidget


class PandoraPlugin(Plugin):

    def __init__(self, context):

        super(PandoraPlugin, self).__init__(context)
        self.setObjectName('Pandora_Gui')

        self.widget_ = MainWidget(self)
        self.widget_.setWindowTitle("Pandora Gui")
        if context.serial_number() > 1:
            self.widget_.setWindowTitle(
                "Pandora Plugin" +
                (' (%d)' % context.serial_number()))

        context.add_widget(self.widget_)

    def shutdown_plugin(self):
        self.widget_.shutdown_plugin()
