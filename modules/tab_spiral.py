from math import ceil, pi
from typing import override

import adsk.core, adsk.fusion

from .lib import fusion_helper as fh
from . import command
from .lib import spline
from .locales import LOCALE

l = LOCALE.spiral


class TabInput(fh.TabInput[command.Command]):
    id = "spiral_tab"
    name = l.spiral

    # parent: command.Command
    # tab: adsk.core.TabCommandInput

    angle: adsk.core.ValueCommandInput
    radii: adsk.core.TextBoxCommandInput
    height: adsk.core.ValueCommandInput
    flip: adsk.core.BoolValueCommandInput
    spline: adsk.core.BoolValueCommandInput

    @override
    def on_created(self, _: adsk.core.CommandCreatedEventArgs, inputs: adsk.core.CommandInputs):
        self.angle = fh.value_control(
            inputs, "spiral_angle", l.angle, "deg", "360", min_exclusive=0
        )
        self.angle.tooltip = l.angle_tooltip
        self.radii = inputs.addTextBoxCommandInput(
            "spiral_radii", l.radii, "10mm, 20mm", 10, False
        )
        self.radii.tooltip = l.radii_tooltip
        self.height = fh.value_control(inputs, "spiral_height", l.height, "mm", "0")
        self.height.tooltip = l.height_tooltip
        self.flip = inputs.addBoolValueInput("spiral_flip", l.flip, True, "", False)
        self.flip.tooltip = l.flip_tooltip
        self.spline = inputs.addBoolValueInput("spiral_spline", l.spline, True, "", False)
        self.spline.tooltip = l.spline_tooltip

    @override
    def on_changed(self, args: adsk.core.InputChangedEventArgs | None):
        # このタブが選ばれたら Detail を非表示にする
        self.parent.details_group.isVisible = not self.tab.isActive

        if not self.tab.isActive:
            return

    @override
    def on_execute_or_preview(self, _: adsk.core.CommandEventArgs, is_preview: bool = False):

        app = adsk.core.Application.get()
        design = adsk.fusion.Design.cast(app.activeProduct)
        comp = design.activeComponent

        sketch = comp.sketches.add(comp.xYConstructionPlane)
        sketch.name = "spiral"
        if is_preview:
            sketch.isConstructionGeometryShown = False
            sketch.arePointsShown = False
            sketch.areProfilesShown = False
            sketch.areDimensionsShown = False
            sketch.areConstraintsShown = False
            sketch.isProjectedGeometryShown = False

        sketch.isComputeDeferred = True
        try:
            units_mgr = adsk.core.UnitsManager.cast(design.unitsManager)
            eval_length = lambda r: units_mgr.evaluateExpression(r, units_mgr.defaultLengthUnits)

            angle = self.angle.value
            radii = [eval_length(r) for r in self.radii.text.split(",")]
            height = self.height.value

            # 点が１つしか与えられていなければ同じ値で複製する
            if len(radii) == 1:
                radii = radii + radii

            # Flip が指定されていれば逆順にする
            if self.flip.value:
                radii = list(reversed(radii))

            n_min = 36 * angle / (2 * pi)  # 10 度ごと
            if len(radii) < n_min:
                m = ceil((n_min - 1) / (len(radii) - 1))
                n = m * (len(radii) - 1) + 1
                # radii の点間を m 等分して補間する
                if self.spline.value and len(radii) >= 5:
                    # スプラインで補間する
                    periodic = abs(angle - 2 * pi) < 1e-3 and radii[0] == radii[-1]
                    interpolate = spline.interpolate(
                        [i * m for i in range(len(radii))], radii, periodic
                    )
                    radii = [interpolate(i) for i in range(n)]
                else:
                    radii = [
                        radii[i // m] + (radii[i // m + 1] - radii[i // m]) * (i % m) / m
                        for i in range(n - 1)
                    ] + [radii[-1]]

            curve = sketch.sketchCurves.sketchFittedSplines.add(
                fh.collection(
                    fh.point3d_polar(radii[i], i * angle / (n - 1), i * height / (n - 1))
                    for i in range(n)
                )
            )

            if not is_preview:
                # fix everything
                for p in curve.fitPoints:
                    p.isFixed = True
                curve.isFixed = True

        finally:
            sketch.isComputeDeferred = False
