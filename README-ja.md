# Fusion 360 用の歯車生成スクリプトです

このスクリプトを使うと Fusion 360でさまざまな歯車を生成できます。

主に学習目的で、3D プリンターで歯車を出力する用途で使われることを想定しています。

<a href="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/assets/spur.gif"><img src="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/assets/spur.gif" width="200"/></a>
<a href="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/assets/helical.gif"><img src="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/assets/helical.gif" width="200"/></a>
<a href="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/assets/internal.gif"><img src="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/assets/internal.gif" width="200"/></a>
<a href="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/assets/rack.gif"><img src="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/assets/rack.gif" width="200"/></a>
<a href="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/assets/worm+wheel.gif"><img src="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/assets/worm+wheel.gif" width="200"/></a>
<a href="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/assets/bevel2.gif"><img src="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/assets/bevel2.gif" width="200"/></a>
<a href="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/assets/crown.gif"><img src="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/assets/crown.gif" width="200"/></a>
<a href="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/assets/archimedean.gif"><img src="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/assets/archimedean.gif" width="200"/></a>

## このスクリプトでできること:

* 様々な歯車の生成
  * 平歯車・はすば歯車
  * 内歯車（はすばも可能）
  * ラック（はすばも可能）
  * ウォームギアとウォームホイール
  * かさ歯車（まがり歯、内歯も可能）
  * クラウンギア（はすばも可能）
  * らせん状の曲線（カムやスプリングの作成に使用）

* 豊富なカスタマイズ項目
  * バックラッシュ
  * 転位量
  * 圧力角
  * 歯末、歯元の丈
  * フィレット径
  * 歯先の延長（ホブ加工用）

* 正確な歯形
  * 円柱形状歯車（平歯車、はすば歯車、内歯車）に対して、このスクリプトが生成する歯元のフィレット部分は通常の歯車の切削工程で生まれるトロコイド曲線を正しく再現します。それに伴い、小歯数の歯車で生じる切り下げも正しく反映します。標準の Spur Gear スクリプトはこれに対応していません。<br>
  下図は青がこのスクリプトで、黄緑が標準の Spur Gear スクリプトで生成した歯形です。標準の Spur Gear スクリプトで生成した歯車は大歯数歯車との組み合わせでは干渉を生じるため正しく回転しません。<br>
  <a href="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/assets/fillet1.png"><img src="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/assets/fillet1.png" width="200"/></a>
  <a href="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/assets/fillet2.png"><img src="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/assets/fillet2.png" width="200"/></a>
  * ウォームホイールやクラウンギア（フェースギア）に対しては、このスクリプトは内部で実際にディスク状あるいはドーナツ状の部材をウォームギア状あるいはピニオンギア状の切削具により切削し、歯車を成形する過程をシミュレーションすることにより、正しい歯車形状を求めています。
  * かさ歯車に対しては、このスクリプトは平歯車の歯形を流用するのではなく、球面インボリュート曲線を用いた正確な歯形を生成します。また歯元には球面トロコイド曲線による自然なフィレットと必要十分な切り下げも実現します。

# 作者

Osamu Takeuchi <osamu@big.jp>

# 免責

このスクリプトは主に自分の学習用に作られたものです。

生成される歯車の形状の正確性は保証されません。

# ライセンス

MIT

# 謝辞

以下のサイトを参考にさせていただきました。大変感謝しています。

- 歯車好きの元クルマエンジニア さんの はてなブログ 「歯車のハナシ」<br>
  https://involutegearsoft.hatenablog.com/

- KHK 小原歯車工業株式会社 さんのホームページ<br>
  https://www.khkgears.co.jp/gear_technology/guide_info.html

- chromia さんの Qiita ページ 「歯車を描く」<br>
  https://qiita.com/chromia/items/629311346c80dfd0eac7

# サブモジュール

- ドキュメント /docs : https://github.com/osamutake/fusion360-study-gears-docs
- 計算機 /calc : https://github.com/osamutake/fusion360-study-gears-calc
- 汎用ライブラリ /modules/lib/fusion_helper : https://github.com/osamutake/fusion360-helper

# ダウンロードとインストール

[最新リリースページ](https://github.com/osamutake/fusion360-study-gears/releases) から zip ファイルをダウンロードし、zip ファイル内の `study-gears` フォルダーを `Fusion 360 API Scripts` フォルダーに配置します。Windows 環境の場合、`C:\Users\(ユーザー名)\AppData\Roaming\Autodesk\Autodesk Fusion 360\API\Scripts` です。

# 基本的な使い方

1. `Fusion 360` の `Design` 環境で `Ctrl+S` を押して `Scripts and Add-ins` ダイアログを表示します。<br><br>
  <a href="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/assets/script-and-addin.png"><img src="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/assets/script-and-addin.png" width="200"/></a><br><br>
2. `study-gears` を選択し、`Run` ボタンを押すか、`study-gears` をダブルクリックしてスクリプトのフロントパネルを起動します。<br><br>
  <a href="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/assets/tab_cylinder.png"><img src="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/assets/tab_cylinder.png" width="200"/></a><br><br>
3. パラメータを変更せず `Ok` ボタンを押せば平歯車が生成されます。<br><br>
<img src="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/assets/first-spur-gear.jpg" width="300"/>

# 歯車設計計算機

歯車の設計に役立つ各種計算をブラウザ上で行える計算機を同梱しています。

[歯車設計計算機](./calc.html)

- 平歯車・はすば歯車: 軸間距離を計算
- 平歯車・はすば歯車: 中心間距離から転位量を計算
- 内歯車: 軸間距離を計算
- 内歯車: 中心距離から転位を計算
- かさ歯車

に関する計算を行えます。

# ５つのタブの紹介

このスクリプトのフロントパネルは５つのタブから構成されており、それぞれのタブで異なる種類の歯車を生成できます。

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

`Bevel` タブで、任意の軸角度を持つベベルギアを生成できます。はすばや内歯にすることも可能です。

<img src="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/assets/bevel.gif" height="100"/><img src="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/assets/bevel2.gif" height="100"/><img src="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/assets/bevel6.gif" height="100"><img src="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/assets/bevel26.gif" height="80">

## Crown タブ

`Crown` タブで、クラウンギア（フェースギア）を生成できます。

<img src="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/assets/crown.gif" height="100"/><img src="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/assets/crown18.gif" height="100"/>

## Spiral タブ

`Spiral` タブで、らせん曲線やカム形状を生成できます。

<img src="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/assets/spiral1.png" width="120"/><img src="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/assets/spiral2.png" width="120"/><img src="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/assets/spiral3.png" width="120"/>

# チュートリアル

- [歯車の基礎知識](https://github.com/osamutake/fusion360-study-gears-docs/blob/master/basics-ja.md)
- [パラメータリファレンス](https://github.com/osamutake/fusion360-study-gears-docs/blob/master/parameters-ja.md)
- [平歯車](https://github.com/osamutake/fusion360-study-gears-docs/blob/master/spur-gear-ja.md)
- [はすば歯車](https://github.com/osamutake/fusion360-study-gears-docs/blob/master/helical-ja.md)
- [ねじ歯車](https://github.com/osamutake/fusion360-study-gears-docs/blob/master/screw-ja.md)
- [内歯車](https://github.com/osamutake/fusion360-study-gears-docs/blob/master/internal-ja.md)
- [ラック＆ピニオン](https://github.com/osamutake/fusion360-study-gears-docs/blob/master/rack-ja.md)
- [ウォーム＆ウォームホイール](https://github.com/osamutake/fusion360-study-gears-docs/blob/master/worm_and_wheel-ja.md)
- [かさ歯車](https://github.com/osamutake/fusion360-study-gears-docs/blob/master/bevel-ja.md)
- [クラウンギア（フェースギア）](https://github.com/osamutake/fusion360-study-gears-docs/blob/master/crown-ja.md)
- [つるまきたが歯車](https://github.com/osamutake/fusion360-study-gears-docs/blob/master/archimedean-ja.md)
- [ゼネバ歯車](https://github.com/osamutake/fusion360-study-gears-docs/blob/master/geneva-ja.md)
- [遊星歯車](https://github.com/osamutake/fusion360-study-gears-docs/blob/master/planetary-ja.md)

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
<img title="ねじ歯車" src="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/assets/screw-gear10.gif" height="100"/>
</a>
<a href="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/internal-ja.md">
<img title="内歯車" src="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/assets/internal.gif" height="100"/>
</a>
<a href="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/rack-ja.md">
<img title="ラック＆ピニオン" src="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/assets/rack.gif" height="100"/>
</a>
<a href="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/worm_and_wheel-ja.md">
<img title="ウォーム＆ホイール" src="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/assets/worm+wheel.gif" height="100"/>
</a>
<a href="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/bevel-ja.md">
<img title="かさ歯車" src="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/assets/bevel2.gif" height="100"/>
</a>
<a href="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/crown-ja.md">
<img title="クラウンギア" src="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/assets/crown.gif" height="100"/>
</a>
<a href="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/archimedean-ja.md">
<img title="つるまきたが歯車" src="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/assets/archimedean.gif" height="100"/>
</a>
<a href="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/geneva-ja.md">
<img title="ゼネバ歯車" src="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/assets/geneva8.gif" height="100"/>
</a>
<a href="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/planetary-ja.md">
<img title="遊星歯車" src="https://github.com/osamutake/fusion360-study-gears-docs/blob/master/assets/planetary25.gif" height="100"/>
</a>
