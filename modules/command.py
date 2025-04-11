from math import pi
from typing import override
import webbrowser

import adsk.core, adsk.fusion

from .lib import fusion_helper as fh
from .locales import LOCALE

l = LOCALE.details


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
        cmd.setDialogMinimumSize(350, 200)

        self.details_group = cmd.commandInputs.addGroupCommandInput("details", l.details)
        self.details_group.isExpanded = False
        details = self.details_group.children

        self.pressure_angle = fh.value_control(
            details,
            "pressure_angle",
            l.pressure_angle,
            "deg",
            "20",
            min=10 / 180 * pi,
            max=32 / 180 * pi,
            tooltip=l.pressure_angle_tooltip,
        )
        self.backlash = fh.value_control(details, "backlash", l.backlash, "mm", "0.05")
        self.backlash.tooltip = l.backlash_tooltip
        self.shift = fh.value_control(details, "shift", l.shift, "", "0.0", min=-0.5, max=2.0)
        self.shift.tooltip = l.shift_tooltip
        self.fillet = fh.value_control(details, "fillet", l.fillet, "", "0.4", min=0, max=0.4)
        self.fillet.tooltip = l.fillet_tooltip
        self.addendum = fh.value_control(
            details, "addendum", l.addendum, "", "1.00", min=0.5, max=2.0
        )
        self.addendum.tooltip = l.addendum_tooltip
        self.dedendum = fh.value_control(
            details, "dedendum", l.dedendum, "", "1.25", min=0.5, max=2.0
        )
        self.dedendum.tooltip = l.dedendum_tooltip
        self.r_clearance = fh.value_control(
            details,
            "r_clearance",
            l.r_clearance,
            "",
            "0.25",
            min=0,
            max=1,
        )
        self.r_clearance.tooltip = l.r_clearance_tooltip
        self.tip_fillet = fh.value_control(
            details,
            "tip_radius",
            l.tip_fillet,
            "",
            "0",
            min=0,
            max=1,
        )
        self.tip_fillet.tooltip = l.tip_fillet_tooltip
        self.show_document = cmd.commandInputs.addBoolValueInput(
            "show_document", l.show_document, False
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
