# slam

| chapter06                                                                                                 | chapter07                                                                                                 | chapter08                                                                                                 | chapter09                                                                                                 | chapter10                                                                                                 |
|:---------------------------------------------------------------------------------------------------------:|:---------------------------------------------------------------------------------------------------------:|:---------------------------------------------------------------------------------------------------------:|:---------------------------------------------------------------------------------------------------------:|:---------------------------------------------------------------------------------------------------------:|
| [![Build Status](https://travis-ci.org/soblin/slam.svg?branch=chapter06)](https://travis-ci.org/soblin/slam) | [![Build Status](https://travis-ci.org/soblin/slam.svg?branch=chapter07)](https://travis-ci.org/soblin/slam) | [![Build Status](https://travis-ci.org/soblin/slam.svg?branch=chapter08)](https://travis-ci.org/soblin/slam) | [![Build Status](https://travis-ci.org/soblin/slam.svg?branch=chapter09)](https://travis-ci.org/soblin/slam) | [![Build Status](https://travis-ci.org/soblin/slam.svg?branch=chapter10)](https://travis-ci.org/soblin/slam) |
|                                                                                                           |                                                                                                           |                                                                                                           |                                                                                                           |                                                                                                           |

---

『SLAM入門 ロボットの自己位置推定と地図構築の技術』の実装レポジトリ.

**著者** 友納 正裕

**出版社** オーム社

**発行年** 2018/03/05

**ISBN** 978-4-274-22166-8

## ゼロから再実装

ROSのRvizとの連携まで進めたいと思っているのだが, そもそも本書のサンプルプログラムは分かりにくい. 理由は

- すべての章のプログラムを1つにまとめている
- 最適化計算や地図のサイズなどのパラメータがソースコード中に分散してハードコーディングされていている. 修正がしにくいし, 自分でも構成が理解しづらい
- 論理時刻が各クラスで重複している

 など. ブラックボックスのまま表面だけをいじるのは嫌なので, サンプルプログラムを見ながらスクラッチから再実装した. 気をつけたのは
 
 - **インクリメンタル** に実装する. 第n章の実装は, 第n章で必要なクラスのみで構成すること.
 - パラメータの定義は1つの箇所で集中して行う. `ParamServer` なるクラスを用意して, `ParamServer::Set("map_size", 40), map_sizr = ParamServer::Get("map_size")` の形でパラメータを得る. ROSのパラメータサーバーに似ている. 論理時刻も `CounterServer::Increment(), CounterServer::Get()` の形でインクリメントしたり取得したりしている. 当然シングルトンクラスになっている.
 - ユニットテストを行うこと. 第6章, 7章で必要になるクラスの多くはテストケースを用意して機能をチェックしている. テストフレームワークとしては [Gtest](https://github.com/google/googletest) を用いている. 各章ごとに, travis-ci 上でテストを行っている.
 - その他C++に慣れること. CMakeでビルドする際に自動で `clang-format` で各ファイルをフォーマットしている.
 
最適化計算での閾値や地図のサイズなどのパラメーターのノミナル値は, すべて `include/slam/parameters.h` に定義を集中している.

## License

MPL-2.0

## Original

https://github.com/furo-org/LittleSLAM

---


