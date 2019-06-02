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

ハイパーパラメータをyamlに書いておいて, それを読み込む形式にしたい. 今の所かなり分散しているし, まとまりがついていない感じがする.

- geometry/Pose2D なし
- geometry/ScanPoint2D なし
- geometry/Scan2D MAX_SCAN_RANGE(6), MIN_SCAN_RANGE(0.1)
- geometry/PointCloudMap AddPoints()におけるskip, MAX_POINT_NUM
- io/SensorDataReader angle_offset(180)
- io/MapDrawer {x, y}{min, max}の初期値, DrawGPのskip(point cloud, robot pose)
- io/SlamLauncher Run()におけるsleep時間, usleep

面倒だしやっぱりいいや.

## 6/2

DataAssociatorLSをテストしようとしているが, テストデータを作るのが面倒くさい. あと, privateメンバにアクセスするために`friend class DataAssociatorLSTestFriend`を外部から利用できるようにした. これはプライベートをテストする際よくあることなのかもしれない. GoogleのProtobufにもそんな記述があった.
