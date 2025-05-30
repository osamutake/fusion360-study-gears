# Change Log

## 20250425

- 間欠歯車に関する記述を追加

## 20250420

- ラック形状を修正
- コードのリファクタリング＆コメント拡充

## 20250418

- チュートリアルにカム機構を追加
- 遊星歯車、つるまきたが歯車の記述を改善

## 20250413

- かさ歯車のタブに設計球半径と基準円錐角を表示するようにした
- ダイアログ表示言語の自動切換えのバグを除去
- チュートリアルに遊星歯車を追加

## 20250412

- 内歯のかさ歯車を最後に反転するようにした
- ダイアログを日本語と英語で自動切換えするようにした

## 20250410

- 内歯のかさ歯車を生成可能になった

## 20250409

- クラウンギア
  - アンダーカット部分とインボリュート部分とを別々に計算することでスプライン曲線による近似制度を高めた
  - その際、もともと相手の歯車と接触しないアンダーカット部分を少しだけ深く切削するようにして、インボリュート部分の計算結果とほとんど一致してしまう部分をなくして切削用のブール演算の負担や計算結果に現れる継ぎ目の揺らぎを軽減した
- 歯幅の広いウォームホイールでエラーが出ないようにした
  - 逆正弦関数の中身が 1 を超えないよう制限した
  - 歯溝形状の計算を歯先円よりも 0.3 x モジュール 分だけ外側まで行い、歯溝が歯先円を抜ける部分をより正確に再現できるようにした
  - 歯車の厚さと計算ステップとがうまく合わないと歯車の端で歯溝が途切れてしまう不具合が残っている（歯車の幅を調節すると回避できる）
- 計算に時間がかかるクラウンギアとウォームホイールについて、計算中のカメラ位置を計算途中を確認できる位置に移動するようにした
- ソースコードの記述を整え、コメントを追加した
- README.html を開いた際に、ブラウザの設定で日本語が優先されている場合に自動的に README-ja.html に飛ぶようにした
- つるまきたが歯車、ゼネバ歯車の説明を追加、その他のドキュメントも少し修正した

## 20250407
- はすばクラウンギアを生成できるようになった
- Spiral タブにスプライン曲線のチェックボックスを追加した

## 20250403
- クラウンギアを生成できるようになった
- ドキュメントを表示するボタンを追加した

## 20250401
- 球面インボリュートに基づくかさ歯車を生成できるようになった

## 20250326
- 内歯車、ラック＆ウォームに関するドキュメントを追加
- zip ファイル内のフォルダ名を修正

## 20250326
- プロジェクトに歯車計算機を追加
- ドキュメントにパラメータ説明、はすば歯車、ネジ歯車を追加

## 20250320
- GitHub へ公開

## 20250319
- 精度向上のためウォームとはすば歯車をロフトで作るようにした
- ウォームホイールを正しく作れるようになった
- ラックの歯先フィレットを付ける位置を調整した
- スケッチの編集時に isComputeDeferred = True して高速化
- コマンド入力を保存する際にダブりがあったのを直した
- タブごとのコードを切り分けた
- ソースコードのフォルダ構造を整理した

## 20250310
- ジョイント構造を単純化した
- コンポーネントに歯数などの情報を含めた
- かさ歯車を90度以上の組み合わせ角度でも生成可能にした

## 20250308
- アクティブコンポーネントの下に歯車を作るようにした
- コマンド入力値が毎回失われないようにした
- らせんを描く機能を追加した(Spiralタブ)
- タイムラインのグループ化が失敗しないようにした

## 20250305
- 平歯車、ラック、ウォームの歯先をフィレット付きで伸ばせるようにした
- 精度を高めるためウォームの生成方法を変更した
- はすば歯車の中央に線が入らないようにした
- はすばラックの生成位置がずれていたのを直した
- 入力コントロールのタブごとにコードを分離した
- vector を 3D 対応にした
- mypy を通せるようにした

## 20250222
- フィレット形状がおかしくなる問題を解決
- 基準円表示に転位量を加えない
- ラックに基準線を入れるようにした

## 20250220
- Details で Radial Clearance を設定可能にした
- 各種処理を fusion_helper に移してコードを読みやすくした

## 20250218B
- はすばかさ歯車が作れるようになった
- かさ歯車の歯幅の指定がバグっていたのを直した

## 20250218
- すぐ歯のかさ歯車が作れるようになった
- ウォームホイールの位置をキャプチャするようにした
- ウォームホイールの切削ホブ径倍率を追加した

## 20250217
- シフト量をモジュール単位で入力するようにした
- ウォームホイールの入力項目を整理した
- ウォームはラックを回転させて作るようにした
- ウォームホイールを回転＆直進するラックで切るようにした

## 20250214B
- ピニオンと組み合わせやすいようにラックの生成位置を調整した
- ラックにバックラッシュが適用されるようにした
- ラック/ウォームの切り替え時の入力値の変化をスムーズにした

## 20250214
- はすば歯車形状を間違っていたので修正
- はすば歯車を回転スイープで押し出すようにした
- 歯形を求める際の歯切りラック形状を生成しそこなうことがあったので修正

## 20250213C
- 転位量を mm で指定するようにした
- 負のバックラッシュを指定できるようにした(歯当たり検証用)

## 20250213B
- バックラッシュを２倍大きく取っていたので修正

## 20250213
- 初出
