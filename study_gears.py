# Author - Osamu Takeuchi <osamu@big.jp>
# Description - This script is to study gear shapes.

import importlib
import pathlib
import traceback

from .modules import command
from .modules import gear_bevel
from .modules import gear_curve
from .modules import gear_cylindrical
from .modules import gear_rack
from .modules import gear_worm_wheel
from .modules import gear_worm
from .modules import tab_bevel
from .modules import tab_cylindrical
from .modules import tab_rack_worm
from .modules import tab_spiral
from .modules.lib import curve
from .modules.lib import function
from .modules.lib import fusion_helper as fh
from .modules.lib import segment
from .modules.lib import spline


def run(_context):

    try:
        # Explicitly reload all modules during development,
        # as they are not automatically reloaded when edited.

        git = pathlib.Path(__file__).resolve().parent / ".git"
        if git.exists():
            importlib.reload(curve)
            importlib.reload(function)
            importlib.reload(fh)
            importlib.reload(segment)
            importlib.reload(spline)

            importlib.reload(command)
            importlib.reload(gear_bevel)
            importlib.reload(gear_curve)
            importlib.reload(gear_cylindrical)
            importlib.reload(gear_rack)
            importlib.reload(gear_worm_wheel)
            importlib.reload(gear_worm)
            importlib.reload(tab_bevel)
            importlib.reload(tab_cylindrical)
            importlib.reload(tab_rack_worm)
            importlib.reload(tab_spiral)

        # read information from the manifest file
        manifest = fh.read_script_manifest(__file__)

        script_name = pathlib.Path(__file__).resolve().name.removesuffix(".py")
        command.Command(
            script_name,
            script_name,
            dict(manifest["description"]).get(""),
            [
                tab_cylindrical.TabInput,
                tab_rack_worm.TabInput,
                tab_bevel.TabInput,
                tab_spiral.TabInput,
            ],
        )

    except:
        fh.message_box(traceback.format_exc())
