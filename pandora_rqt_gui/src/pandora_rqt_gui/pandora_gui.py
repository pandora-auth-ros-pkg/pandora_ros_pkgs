from qt_gui.plugin import Plugin
from .main_widget import MainWidget

from std_msgs.msg import String

class PandoraPlugin(Plugin):


    def __init__(self, context):
        super(PandoraPlugin, self).__init__(context)
        self.setObjectName('Pandora_Gui')

        self._widget = MainWidget(self)
       
         
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
            
            
        context.add_widget(self._widget)
       

    def shutdown_plugin(self):
        self._widget.shutdown_plugin()


