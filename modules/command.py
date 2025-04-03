from math import pi
from typing import override
import webbrowser

import adsk.core, adsk.fusion

from .lib import fusion_helper as fh


class Command(fh.TabbedCommand):
    details_group: adsk.core.GroupCommandInput
    backlash: adsk.core.ValueCommandInput
    pressure_angle: adsk.core.ValueCommandInput
    dedendum: adsk.core.ValueCommandInput
    addendum: adsk.core.ValueCommandInput
    r_clearance: adsk.core.ValueCommandInput
    fillet: adsk.core.ValueCommandInput
    shift: adsk.core.ValueCommandInput
    tip_fillet: adsk.core.ValueCommandInput
    show_document: adsk.core.BoolValueCommandInput

    @override
    def on_created(self, args: adsk.core.CommandCreatedEventArgs):
        # create the tabs first
        super().on_created(args)

        # then add the details group
        cmd = adsk.core.Command.cast(args.command)

        # set minimum width of the dialog to show all tabs
        cmd.setDialogMinimumSize(300, 200)

        self.details_group = cmd.commandInputs.addGroupCommandInput("details", "Details")
        self.details_group.isExpanded = False
        details = self.details_group.children

        self.pressure_angle = fh.value_control(
            details,
            "pressure_angle",
            "Pressure Angle",
            "deg",
            "20",
            min=10 / 180 * pi,
            max=32 / 180 * pi,
        )
        self.backlash = fh.value_control(details, "backlash", "Backlash", "mm", "0.05")
        self.backlash.tooltip = "The backlash between gears with module as unit length."
        self.pressure_angle.tooltip = "The pressure angle of the gear (default = 20 deg)."
        self.shift = fh.value_control(details, "shift", "Shift", "", "0.0", min=-0.5, max=2.0)
        self.shift.tooltip = "The shift of the rack tool with module as the unit."
        self.fillet = fh.value_control(details, "fillet", "Fillet", "", "0.4", min=0, max=0.4)
        self.fillet.tooltip = "The fillet radius of the rack tool with module as the unit."
        self.addendum = fh.value_control(
            details, "addendum", "Addendum", "", "1.00", min=0.5, max=2.0
        )
        self.addendum.tooltip = (
            "Distance from the pitch circle to the tip circle with module as the unit."
        )
        self.dedendum = fh.value_control(
            details, "dedendum", "Dedendum", "", "1.25", min=0.5, max=2.0
        )
        self.dedendum.tooltip = (
            "Distance from the pitch circle to the root circle with module as the unit."
        )
        self.r_clearance = fh.value_control(
            details, "r_clearance", "Radial Clearance", "", "0.25", min=0, max=1
        )
        self.r_clearance.tooltip = (
            "The distance between the root circle to the tip of the other gear, "
            + "which determines the maximum fillet diameter."
        )
        self.tip_fillet = fh.value_control(
            details, "tip_radius", "Tip fillet length", "", "0", min=0, max=1
        )
        self.tip_fillet.tooltip = (
            "Tip fillet extruding length"
            + " from the tip circle with module as the unit. Only for hobbing"
        )
        self.show_document = cmd.commandInputs.addBoolValueInput(
            "show_document", "Show document", False, initialValue=False
        )

    @override
    def on_changed(self, args: adsk.core.InputChangedEventArgs | None):
        self.tip_fillet.isEnabled = False
        self.shift.isEnabled = True
        self.fillet.isEnabled = True
        self.r_clearance.isEnabled = True

        if self.show_document.value:
            readme = __file__.replace("\\", "/").replace("modules/command.py", "README.html")
            webbrowser.open("file:///" + readme)
        self.show_document.value = False

        super().on_changed(args)

        if not self.tip_fillet.isEnabled:
            self.tip_fillet.value = 0
