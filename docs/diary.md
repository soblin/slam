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

## 6/4

PoseEstimatorICP::EstimatePoseは

1. initPoseとして(おそらく)直前の位置を与えられる. そしてそれをpredictedPoseの初期値として. predictedPoseを更新していく. メンバ変数のm\_cur\_scanをDatAssociatorのFindCorrespondence()において利用する. そしてそのマッチング結果をm\_optmizerにセットして最適化された新しい姿勢newPoseを得る. この繰り返し.


## 6/11

SlamFrontEnd::Process()が何をやっているのかまだきちんと理解していない. 

1. まず始めにScanMatcher2DのMatchScanを実行する. ScanMatcher2Dは直前のスキャン結果を保持している(m\_prev\_scan).それはodoMotinを求めるため.
2. 一方PointCloudMapはICPによる推定位置を保持している. そこで, ICPによる推定位置の初期値を, 前回の推定位置からodoMotionだけ移動した点とする.
3. 次にMakeRefScanする. 今はMakeRefScanMakerBS::MakeRefScanを呼んでいるので, 前回のスキャン(m\_prev\_scanあるいは今はPointCloudMapBS::last_scanと同じ)をグローバルフレームに変換している. 
4. そのrefScan(グローバルフレーム)とcurScan(ローカルフレーム)をPoseEsitmatorICPにセットする. この2つが一致するようにcurScanの属する姿勢を最適化により求める. このときDataAssociatorLS::SetRefBaseを呼んでおり, 次にFindCorrespondenceすることでマッチング結果を得る. FindCorrespondence()は毎回呼ばれている.
5. いよいよPoseEstimatorICPのEstimatePoseを呼ぶ. この中身を説明する. まず始めに現在の姿勢の推定値に基づいてFindCorrespondenceを呼ぶ. そこでcurScanを推定値に基づいたグローバル座標に写し, それと参照スキャンとのマッチングを行う. そしてOptimizer::SetPointsによりAssosiatorの対応が取れた点群のマッチング結果をOptimizerに渡す. そして一段階最適化した結果を得る.
5. ScanMatcher2D::GrowMapにより現在スキャンcurScanを推定位置estimatedPoseに基づいてグローバル座標に変換し, PointCloudMapに登録する.

## 6/18

8章に入った. 7章はICPを行っていて少し複雑になってしまったが, 結局の所6章との違いは, curScanに含まれる姿勢の推定値である. 依然, スキャンで得られた点を全て用いている点は何も変わらない. 具体的には一つ前のスキャン結果(参照スキャン)と姿勢の推定値を保存しておき, 現在のスキャンとのICPを行うことで現在の姿勢の推定値を得る. その際, 現在の姿勢の推定値の初期として, 一つ前の姿勢の推定値からオドメトリに基づく移動を足した値を用いる. それを実現するために, 

## 6/30
NNGridTableのテーブルを確保する際に, サイズをm\_table\_size * m\_table\_size にしていたため, 地図が表示されていなかった. まだ途中で計算が止まっている(おそらく最適化計算のwhileが終了していないため)ところもあるが, 一応動いている.

次にすべてのクラスのコンストラクタ, デストラクタをヘッダーに移す. そしてconstなメンバ変数は宣言時に初期化することにしよう.
