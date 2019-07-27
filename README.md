# slam

| chapter06                                                                                                 | chapter07                                                                                                 | chapter08                                                                                                 | chapter09                                                                                                 | chapter10                                                                                                 |
|:---------------------------------------------------------------------------------------------------------:|:---------------------------------------------------------------------------------------------------------:|:---------------------------------------------------------------------------------------------------------:|:---------------------------------------------------------------------------------------------------------:|:---------------------------------------------------------------------------------------------------------:|
| [![Build Status](https://travis-ci.org/soblin/slam.svg?branch=chapter06)](https://travis-ci.org/soblin/slam) | [![Build Status](https://travis-ci.org/soblin/slam.svg?branch=chapter07)](https://travis-ci.org/soblin/slam) | [![Build Status](https://travis-ci.org/soblin/slam.svg?branch=master)](https://travis-ci.org/soblin/slam) | [![Build Status](https://travis-ci.org/soblin/slam.svg?branch=chapter08)](https://travis-ci.org/soblin/slam) | [![Build Status](https://travis-ci.org/soblin/slam.svg?branch=master)](https://travis-ci.org/soblin/slam) |
|                                                                                                           |                                                                                                           |                                                                                                           |                                                                                                           |                                                                                                           |

---

『SLAM入門 ロボットの自己位置推定と地図構築の技術』の実装レポジトリ.

サンプルプログラムはすべての章のプログラムを1つにまとめていて分かりにくいので, 各章ごとにブランチを切って自分でスクラッチから実装しなおした. またオリジナルの実装には i)パラメータがハードコーディングされていて非常に修正がしにくい, ii)論理時刻が多くのクラスで重複していて無駄になっているという問題があるので, `ParameterServer`, `CounterServer` なるシングルトンクラスを作って, 各クラスは必要な値をそこから受け取るように設計した. 本当は `parameters.h` の値を constexpr に利用したかったが, 今回はオーバーヘッドを生むことを承知で, 毎回 `ParameterServer` から値を受け取るようにしている.

最適化計算での閾値や地図のサイズなどのパラメーターのノミナル値は, すべて `include/slam/parameters.h` に定義を集中している.


**著者** 友納 正裕

**出版社** オーム社

**発行年** 2018/03/05

**ISBN** 978-4-274-22166-8

---


