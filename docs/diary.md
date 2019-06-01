## 5/29

C++としての構成をllvmらしく綺麗に分ける. 面倒だが今後のためにも役立つ. とりあえずmainから作ってみる.

## 5/29

なんとか形だけはらしくらってきた. cmakeでExternal\_Projectを加えたりするのが楽なので, 徐々に大きくしていくのが楽になりそう.

Pose2Dの設計はデータの一貫性が問われるケーススタディであると思う. というのはthetaの変更と回転行列の変更は必ず同期しなければならないからだ.

## 6/1

データ構造がいくつかあってややこしい. 画像にまとめた.

`lib/geometry/Pose2D.cpp` の `normalize` で定義しているように, 角度は (-M\_PI, M_PI] に収めるようにしている.

SlamLauncherがSetFilenameでテストファイルをオープンする時, その動作はSensorDataReaderのOpenScanFileが行う. そしてLoadScan(size_t cnt_id, Scan2D &output)よりoutputへとその行の姿勢とスキャンデータを保持していく.

SlamLauncherのm_mapが保持されるデータの全て. 基本的に(1)ファイルを1行読んで (2)Scan2D型としてSensorDataReader内部で一旦バッファリングされたあと (3)MapByOdometry()によりPointCloudMapへと観測された点は保存されていく.
