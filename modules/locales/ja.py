from .en import Locale

LOCALE = Locale()

d = LOCALE.details

d.details = "詳細設定"
d.pressure_angle = "圧力角"
d.backlash = "バックラッシュ"
d.shift = "転位量"
d.fillet = "フィレット"
d.addendum = "歯末の丈"
d.dedendum = "歯元の丈"
d.pressure_angle_tooltip = "歯車の圧力角（デフォルト = 20度）"
d.backlash_tooltip = "回転・並進方向に測ったバックラッシュ"
d.shift_tooltip = "モジュールを単位とした転位量"
d.fillet_tooltip = "歯車切削ツール先端のフィレット半径の最大値(モジュールを単位)"
d.addendum_tooltip = "モジュールを単位とした、基準円から歯先円までの距離（標準値 1.0）"
d.dedendum_tooltip = "モジュールを単位とした、基準円から歯底円までの距離（標準値 1.25）"
d.r_clearance = "頂隙"
d.r_clearance_tooltip = "モジュールを単位とした、基準円から歯先円までの距離（標準値 1.0）"
d.tip_fillet = "歯先フィレット"
d.tip_fillet_tooltip = (
    "モジュールを単位とした、歯先を延長するフィレット押出し長さ。切削ツール形状生成用。"
)
d.show_document = "ヘルプ"

cy = LOCALE.cylindrical
cy.cylindrical = "円筒歯車"
cy.module = "モジュール"
cy.module_tooltip = "モジュールは歯車のピッチをπで割った値"
cy.number_teeth = "歯数"
cy.thickness = "厚み"
cy.helix_angle = "はすば角度"
cy.helix_direction = "はすば方向"
cy.right = "右"
cy.left = "左"
cy.diameter = "穴径あるいは外径"
cy.diameter_tooltip = "内歯車がチェックされている場合は外径、それ以外の場合は穴径"
cy.internal = "内歯車"
cy.worm_wheel = "ウォームホイール"
cy.worm_diameter = "ウォーム径"
cy.worm_spirals = "ウォーム条数"
cy.dp = "基準円直径"
cy.dp_tooltip = "基準円直径"

rw = LOCALE.rack_worm
rw.rack_worm = "ラック/ウォーム"
rw.module = "モジュール"
rw.module_tooltip = "モジュールは歯車のピッチをπで割った値"
rw.thickness = "厚さ/直径"
rw.thickness_tooltip = "ラックの厚さあるいはウォームの直径"
rw.length = "長さ"
rw.helix_angle = "はすば角度"
rw.helix_direction = "はすば方向"
rw.right = "右"
rw.left = "左"
rw.worm_spirals = "ウォーム条数"
rw.worm_spirals_tooltip = "ウォームの条数。ラックの場合は0を指定する。"
rw.height = "高さ"
rw.height_tooltip = "ラックの高さ"

bv = LOCALE.bevel
bv.bevel = "かさ歯車"
bv.module = "モジュール"
bv.module_tooltip = "モジュールは歯車のピッチをπで割った値"
bv.axes_angle = "軸角度"
bv.axes_angle_tooltip = "２つのかさ歯車の軸角度"
bv.n_teeth1 = "歯数1"
bv.n_teeth2 = "歯数2"
bv.width = "歯幅"
bv.spiral_angle = "ねじれ角"

cr = LOCALE.crown
cr.crown = "クラウンギア"
cr.module = "モジュール"
cr.module_tooltip = "モジュールは歯車のピッチをπで割った値"
cr.crown_teeth = "クラウン歯数"
cr.pinion_teeth = "ピニオン歯数"
cr.helix_angle = "はすば角度"
cr.helix_direction = "はすば方向"
cr.right = "右"
cr.left = "左"
cr.outer_ext = "外向き歯幅"
cr.outer_ext_tooltip = "基準円から外向きの歯幅（モジュール単位）"
cr.inner_ext = "内向き歯幅"
cr.inner_ext_tooltip = "基準円から内向きの歯幅（モジュール単位）"

sp = LOCALE.spiral
sp.spiral = "らせん"
sp.angle = "回転角"
sp.angle_tooltip = "らせんの回転角"
sp.radii = "半径"
sp.radii_tooltip = (
    "半径のリスト。カンマで区切る。\n"
    + "指定された半径は回転角に沿って等間隔に配置されます。\n"
    + "間を連続的に補間することでらせんを形成します。"
)
sp.height = "高さ"
sp.height_tooltip = "らせんの高さ。平面的ならせんを作成する場合は0を指定します。"
sp.flip = "反転"
sp.flip_tooltip = "らせんの回転方向を反転します。"
sp.spline = "スプライン補間"
sp.spline_tooltip = (
    "線形補間の代わりにスプライン補間を使用して指定された半径の間を補間します。\n"
    + "指定された半径が 5 つ以上あり、最初と最後が等しい時のみ効力を持ちます。\n"
    + "カム形状を生成するために用いられることを想定しています。"
)
