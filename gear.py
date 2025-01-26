# Author-
# Description-

import adsk.cam
import adsk.core
import adsk.fusion


def run(_):
    app = adsk.core.Application.get()
    ui = app.userInterface
    ui.messageBox("Hello script")
