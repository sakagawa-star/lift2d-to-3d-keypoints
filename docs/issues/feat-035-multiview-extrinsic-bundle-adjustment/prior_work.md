# feat-035 先行研究調査

調査日: 2026-08-17（Web調査）

## 1. 3DGSに対する render-and-match ポーズ精緻化（feat-026 / アンカー項の裏付け）

- **GSLoc / GS-CPR**（2024-2025）: 3DGSからRGBと深度をレンダリングし、実写との2Dマッチングに **MASt3R** を使い、深度で2D-3D対応を作ってポーズを精緻化する。単一カメラのローカライズ用途。feat-026 の「レンダ→マッチ→深度リフト→PnP」と同型で、方式の妥当性を裏付ける。アンカー対応点のマッチャーとして LoFTR の代わりに MASt3R を使う先例でもある。
  - https://openreview.net/forum?id=mP7uV59iJM （GS-CPR）
  - https://www.researchgate.net/publication/383280277_GSLoc_Efficient_Camera_Pose_Refinement_via_3D_Gaussian_Splatting
- **iComMa**（2023）: 3DGSの微分可能レンダリングを逆問題として勾配法でポーズ推定（comparing + matching 損失）。マッチングベースの代替となる勾配ベース路線。
  - https://www.semanticscholar.org/paper/6d5a73f357c6748c02458f749a3cda17dedbfa0b
- **GSplatLoc**（2024）: 3DGSを使った高精度ローカライズ。
  - https://arxiv.org/pdf/2412.20056

**feat-035への示唆**: アンカー項（3DGSレンダ↔実写マッチ→深度リフト）は確立された定石。将来アンカー側マッチャーを LoFTR→MASt3R に替える選択肢もある（現設計は feat-026 実証済みの LoFTR を維持）。

## 2. 固定多カメラのターゲットレス較正（feat-035 本体の直接の先行研究）

- **Robust Multi-view Camera Calibration from Dense Matches**（VISAPP 2026）: **3〜10台の固定カメラ**を、キャリブレーションパターンなしで密マッチング（RoMa）から較正する。**動物行動分析・監視映像**を明示的なユースケースとする点も本プロジェクトと一致。要素技術: サイクル整合性（nn-cyclic distance）による対応選別、三角測量交会角に基づくガウス型スコア（最適角30°）、増分型（逐次ビュー追加+BA）とグローバル型（VGGT初期化→全視点同時BA）の2パイプライン。歪みの強いカメラでも VGGT 単体を大幅に上回る精度（AUC@30°=79.9 vs 40.4）。
  - https://arxiv.org/html/2512.15608

**feat-035への示唆**: (1) 学習ベース密マッチャー + BA が疎な固定リグで機能することの直接的な実証。(2) 対応選別のサイクル整合性チェック、交会角の重み付け（ハード閾値でなくスコア化）は FR-008 実験・将来改良の候補。(3) 相違点: 同論文は内部パラメータも推定し、ワールド座標系は任意。feat-035 は K既知・3DGS座標系アンカーが必須要件で、この部分は先行研究に直接の対応物がない（GSLoc系〔単体〕と VISAPP系〔多視点・アンカーなし〕の組み合わせが feat-035 の新規部分）。

## 3. 画像集合からのグローバルポーズ推定（ブートストラップの代替・フォールバック）

- **MASt3R-SfM**（2024）: MASt3R の密対応を使い、疎・カジュアルな画像集合から全カメラポーズを一括推定。
  - https://anekha.github.io/research/computer%20vision/3d/2025/02/28/MASt3RSfM.html
  - https://learnopencv.com/mast3r-sfm-grounding-image-matching-3d/
- **G-MASt3R-SfM**（2026）: グラフベースのビュー選別と多段最適化で MASt3R-SfM を頑健化。
  - https://arxiv.org/html/2606.22856v1
- **VGGT** / **Pow3R**: 画像集合→ポーズのフィードフォワード推定。Pow3R は既知内部パラメータを補助入力にできる。
  - https://europe.naverlabs.com/research/3d-foundation-models/

**feat-035への示唆**: ペアワイズの貪欲ブートストラップが連鎖的に失敗する場合、8枚一括のグローバル初期化（MASt3R-SfM / VGGT）→ アンカー対応点で3DGS座標系へ相似変換合わせ、という代替初期化経路が文献的に確立している。本案件では Won't（設計は貪欲ブートストラップ）だが、失敗時の次善策として記録する。

## 4. 人物・動物キーポイントによる自己較正（案B。将来案件の文献基盤）

- **Anipose**（2021）: マーカーレス3Dポーズ推定ツールキット。キーポイントからのカメラ較正（BA）を含む。
  - https://pmc.ncbi.nlm.nih.gov/articles/PMC8498918/
- **Online Marker-free Extrinsic Camera Calibration using Person Keypoint Detections**（2022）: 人物2Dキーポイント検出のみから factor graph で複数カメラ外部パラメータをオンライン推定。数分で収束。
  - https://link.springer.com/chapter/10.1007/978-3-031-16788-1_19
- **Multi-Camera Self-Calibration in Sports Motion Capture: Leveraging Human and Stick Poses**（2026）: 人物・スティックのポーズを使った自己較正。
  - https://arxiv.org/html/2604.17567v1
- **SteerPose**（2025）: 外部パラメータ較正と人物マッチングの同時解法。
  - https://www.researchgate.net/publication/392371134
- **lab-camera-dynamic-calibrator**（GitHub）: RTMPose/Metrabs + BA（scipy trf）+ メトリックスケーリングの実装例。
  - https://github.com/flodelaplace/lab-camera-dynamic-calibrator

**feat-035への示唆**: 案B（被写体キーポイントでの継続再較正）は成熟した研究領域で、将来案件の実現可能性は高い。疎でノイジーなキーポイント対応に古典的BAを直接使うと不安定という指摘（Unconstrained Multi-view Human Pose Estimation, 2026: https://arxiv.org/pdf/2604.24312 ）があり、案B実施時は頑健化（factor graph・時系列平滑化）が必要。

## 5. 多カメラ + 3DGS の同時BA（周辺領域）

- **MCGS-SLAM**（2025）: 同期多カメラRGBから3DGSマップを構築し、多カメラBA（光度+幾何残差）でポーズと深度を同時精緻化。SLAM文脈だが「多カメラ同時BA + 3DGS残差」の組み合わせの実例。
  - https://arxiv.org/abs/2509.14191

## まとめ（feat-035 設計への反映判断）

| 知見 | 反映 |
|---|---|
| render+MASt3R+深度リフトの定石性（GSLoc/GS-CPR） | 現設計を裏付け。変更なし |
| 密マッチャー+BAの多視点較正実証（VISAPP 2026） | 現設計を裏付け。変更なし |
| サイクル整合性による対応選別 | **実質採用**: MASt3R 標準の相互最近傍（reciprocal NN）マッチングがペア版に相当（design §2.1 に明記。2026-08-17） |
| 交会角のスコア重み付け（最適30°） | **採用**: クロス項の重み `w_k = exp(-(θ-30°)²/(2σ²))` として design §2.5 に反映。σ は FR-008 実験で決定（2026-08-17） |
| アンカーマッチャーの MASt3R 化（GS-CPR 先例） | FR-008 の任意追加実験項目として design §3 に記録 |
| グローバル初期化（MASt3R-SfM/VGGT) | ブートストラップ連鎖失敗時の代替経路として記録（本案件 Won't。design §5 ADR に判断を記録） |
| 案Bの文献基盤（人物キーポイント較正） | 将来案件の根拠。本案件は変更なし |
