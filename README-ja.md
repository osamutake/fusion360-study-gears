# Fusion 360 用の歯車生成スクリプトです

このスクリプトを使うと Fusion 360でさまざまな歯車を生成できます。

主に学習目的で、3D プリンターで歯車を出力する用途で使われることを想定しています。

<a href="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/assets/spur.gif"><img src="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/assets/spur.gif" width="200"/></a>
<a href="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/assets/helical.gif"><img src="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/assets/helical.gif" width="200"/></a>
<a href="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/assets/internal.gif"><img src="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/assets/internal.gif" width="200"/></a>

<a href="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/assets/rack.gif"><img src="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/assets/rack.gif" width="200"/></a>
<a href="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/assets/worm+wheel.gif"><img src="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/assets/worm+wheel.gif" width="200"/></a>
<a href="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/assets/bevel2.gif"><img src="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/assets/bevel2.gif" width="200"/></a>

## このスクリプトでできること:

* 様々な歯車の生成
  * 平歯車・はすば歯車
  * 内歯車（はすばも可能）
  * ラック（はすばも可能）
  * ウォームギアとウォームホイール
  * かさば歯車（はすばも可能）
  * らせん状の曲線（カムやスプリングの作成に使用）

* 豊富なカスタマイズ項目
  * バックラッシュ
  * 転位量
  * 圧力角
  * 歯末、歯元の丈
  * フィレット径
  * 歯先の延長（ホブ加工用）

また、このスクリプトは歯元のフィレット部分は通常の歯車の切削工程で生まれるトロコイド曲線を正しく再現します。それに伴い、小歯数の歯車で生じる切り下げも正しく反映します。標準の Spur Gear スクリプトはこれに対応していません。

下図は青がこのスクリプトで、黄緑が標準の Spur Gear スクリプトで生成した歯形です。標準の Spur Gear スクリプトで生成した歯車は大歯数歯車との組み合わせでは干渉を生じるため正しく回転しません。

<a href="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/assets/fillet1.png"><img src="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/assets/fillet1.png" width="200"/></a>
<a href="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/assets/fillet2.png"><img src="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/assets/fillet2.png" width="200"/></a>

# 作者

Osamu Takeuchi <osamu@big.jp>

# 免責

このスクリプトは主に自分の学習用に作られたものです。

生成される歯車の形状の正確性は保証されません。

# ライセンス

MIT

# サブモジュール

- /docs : https://github.com/osamutake/fusion360-study-gears-docs
- /modules/lib/fusion_helper : https://github.com/osamutake/fusion360-helper

# ダウンロードとインストール

最新リリースページから zip ファイルをダウンロードし、zip ファイル内の `study-gears` フォルダーを `Fusion 360 API Scripts` フォルダーに配置します。Windows 環境の場合、`C:\Users\(ユーザー名)\AppData\Roaming\Autodesk\Autodesk Fusion 360\API\Scripts` です。

# 基本的な使い方

1. `Fusion 360` の `Design` 環境で `Ctrl+S` を押して `Scripts and Add-ins` ダイアログを表示します。
2. `study-gears` を選択し、`Run` ボタンを押すか、`study-gears` をダブルクリックしてスクリプトのフロントパネルを起動します。
3. パラメータを変更せず `Ok` ボタンを押せば平歯車が生成されます。<br><br>
<img src="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/assets/first-spur-gear.jpg" width="300"/>

# ４つのタブの紹介

このスクリプトのフロントパネルは４つのタブから構成されており、それぞれのタブで異なる種類の歯車を生成できます。

## Cylindrical タブ

 `Cylindrical` タブで、平歯車、はすば歯車、内歯車、ウォームホイールを生成できます。

- `Helix Angle` に非ゼロの値を設定すると、**はすば歯車** が生成されます。
- `Internal` チェックボックスをオンにすると、**内歯車** が生成されます。
  - `Outer Diameter` は歯先円直径より大きく設定して下さい。
- `Worm Wheel` チェックボックスをオンにすると、**ウォームホイール** が生成されます。
    - `Worm diameter` と `Num. Spiral` を適切に設定して下さい。
- それ以外であれば、通常の **平歯車** が生成されます。

<img src="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/assets/helical.gif" width="150"/><img src="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/assets/internal.gif" width="150"/><img src="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/assets/worm+wheel.gif" width="150"/><img src="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/assets/spur.gif" width="150"/>

図は順にはすば歯車、内歯車、ウォームホイール、平歯車です。

## Rack/Worm タブ

`Rack/Worm` タブで、ラックギアとウォームギアを生成できます。

 - `Num. Spiral` に非ゼロの値を設定すると、**ウォームギア** が生成されます。
 - それ以外の場合は、**ラックギア** が生成されます。

<img src="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/assets/worm+wheel.gif" width="150"/><img src="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/assets/rack.gif" width="150"/>

図は左がウォーム＆ウォームホイール、右がはすばのラック＆ピニオンです。

ピニオンは通常のはすば歯車ですが、ウォームホイールはウォームに合わせて切られた特殊形状の歯車になります。

## Bevel タブ

`Bevel` タブで、任意の軸角度を持つベベルギアを生成できます。はすばにすることも可能です。

<img src="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/assets/bevel.gif" width="150"/><img src="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/assets/bevel2.gif" width="150"/>

## Spiral タブ

`Spiral` タブで、らせん曲線やカム形状を生成できます。

<img src="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/assets/spiral1.png" width="120"/><img src="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/assets/spiral2.png" width="120"/><img src="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/assets/spiral3.png" width="120"/>

# チュートリアル

- [歯車の基礎知識](https://github.com/osamutake/fusion360-study-gears-docs/blob/master/basics-ja.md)
- [パラメータリファレンス](https://github.com/osamutake/fusion360-study-gears-docs/blob/master/parameters-ja.md)
- [平歯車](https://github.com/osamutake/fusion360-study-gears-docs/blob/master/spur-gear-ja.md)
- [はすば歯車](https://github.com/osamutake/fusion360-study-gears-docs/blob/master/helical-ja.md)
- [ねじ歯車](https://github.com/osamutake/fusion360-study-gears-docs/blob/master/screw-ja.md)

<a href="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/basics-ja.md">
<img title="歯車の基礎知識" src="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/assets/module.png" height="100"/>
</a>
<a href="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/parameters-ja.md">
<img title="パラメータ詳細" src="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/assets/details.png" height="100"/>
</a>
<a href="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/spur-gear-ja.md">
<img title="平歯車" src="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/assets/spur.gif" height="100"/>
</a>
<a href="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/helical-ja.md">
<img title="はすば車" src="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/assets/helical.gif" height="100"/>
</a>
<a href="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/screw-ja.md">
<img title="ねじ車" src="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/assets/screw-gear10.gif" height="100"/>
</a>
