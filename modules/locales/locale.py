from typing import cast
import adsk.core

from . import en
from . import ja

ZH = adsk.core.UserLanguages.ChinesePRCLanguage
ZH_TW = adsk.core.UserLanguages.ChineseTaiwanLanguage
CS = adsk.core.UserLanguages.CzechLanguage
EN = adsk.core.UserLanguages.EnglishLanguage
FR = adsk.core.UserLanguages.FrenchLanguage
DE = adsk.core.UserLanguages.GermanLanguage
HU = adsk.core.UserLanguages.HungarianLanguage
IT = adsk.core.UserLanguages.ItalianLanguage
JA = adsk.core.UserLanguages.JapaneseLanguage
KO = adsk.core.UserLanguages.KoreanLanguage
PL = adsk.core.UserLanguages.PolishLanguage
PT = adsk.core.UserLanguages.PortugueseBrazilianLanguage
PT_BR = adsk.core.UserLanguages.PortugueseBrazilianLanguage
RU = adsk.core.UserLanguages.RussianLanguage
ES = adsk.core.UserLanguages.SpanishLanguage
TR = adsk.core.UserLanguages.TurkishLanguage

app = adsk.core.Application.get()
locale = cast(int, app.preferences.generalPreferences.userLanguage)

LOCALE = en.LOCALE
match locale:
    case JA:
        LOCALE = ja.LOCALE
