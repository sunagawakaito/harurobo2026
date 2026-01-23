# NUCLEO_446_board_ver1.x_template_vscode

STM32 の NUCLEO-F446RE の拡張ボード (NUCLEO_446_board) ver1.x 用のプロジェクトテンプレート。git で管理しつつ、vscode でビルドできるようにしている。CubeIDE ではビルドできないので注意。

なお、`/build/` や他の環境依存のものを含むものは git の管理対象から除外している。

## vscode での STM32 開発環境の構築

後日 esa などで公開する。

## プロジェクトの作成方法

1. `kurt9dai` の Organization などで新しいリポジトリを作成し、その際、`Start with a template` の項目においてこのリポジトリを選択する。または、このリポジトリのページ右上の `Use this template` のボタンから新規作成する。
1. 作成したリポジトリを `$ git clone --recursive <リポジトリのURL>` でクローンし、vscode で開く。
1. CMake のビルドオプションの選択を要求されたら、`Debug` を選択する。
1. [`/NUCLEO_446_board_ver1.x_template_vscode.ioc`](/NUCLEO_446_board_ver1.x_template_vscode.ioc) のファイル名を `<リポジトリ名>.ioc` に変更し、中身の `ProjectManager.ProjectFileName` の値も `<リポジトリ名>.ioc`に、`ProjectManager.ProjectName` の値を `<リポジトリ名>` に変更する。(これはやらなくてもビルドできるが、管理が大変になるので変更を推奨する。)
1. 拡張機能の STM32Cube の `Setup STM32Cube project(s)` から `Board / Device` などを設定(`Configure`)し、`Save and close` で完了する。([`/.settings/`](/.settings) に設定のファイルが生成、保存される)
1. 初回ビルドを下部 (Status Bar) の CMake Tools の `Build` ボタン、または `F7`キーから行う。

(4, 5 については出来ない、分からない場合は一旦飛ばして 6 のビルドをしてもらって構わない。それでビルドが通ればOK。)

必要に応じて、プロジェクトのために[このドキュメントファイル](/README.md)を書き換えること。

## ビルドの方法 (CMake Tools)

マイコンへの書き込みやデバッグは行わない。

### 方法1. Status Bar の `Build` ボタン

拡張機能の CMake Tools による、Status Bar に表示される `Build` ボタンから行う。また、このデフォルトのショートカットキーは `F7`。

プロジェクトのビルドのみ。前回のビルドからの差分をビルドする。

### 方法2. Task の実行

上部タブから `Terminal > Run Task...` または `Terminal > Run Build Task...`(`Ctrl` + `Shist` + `B`) を選択し、さらに実行する Task を選択する。CMake Tools より提供される Task のうち、必要そうなものを [`/.vscode/tasks.json`](/.vscode/tasks.json) に追加している。現時点 (2025-11-07 時点) で追加しているものは以下の通り。

- `CMake: build`
- `CMake: clean`
- `CMake: clean rebuild`

また、Side Bar の CMake Tools から `PINNED COMMANDS` の `Run Task` によっても同様に実行できる。なお、他の Task を追加したい場合は `Configure Task` や Command Palette などから追加すればよい。

## 書き込み、デバッグの方法 (Run and Debug)

### 方法1. Run and Debug (`F5`)

ビルド、書き込み、デバッグを行う。ブレークポイントを設定できる。なお、デフォルトで `HAL_Init();` で一時停止するようになっている。

### 方法2. Run Without Debugging (`Ctrl` + `F5`)

書き込みまでを行う(と思われる)。

## 注意点1. コード生成時

ピン設定の変更を反映させたいとき、つまりコード生成するとき、以下の手順を踏むこと。

### コード生成前

- [`main.cpp`](/Core/Src/main.cpp) のファイル名を `main.c` に変更する。

### コード生成後

- `main.c` を `main.cpp` に戻す。
- ~~`cmake/stm32cubemx/CMakeLists.txt` 内の以下のような箇所で、`main.c` の箇所を `main.cpp` に変更する。~~ (書き換え不要な設定に変更。(2025-11-13, commit `f4a5cd4`))

## 注意点2. ファイル追加時

コード分割等のためにファイルを追加する時は、基本的に、ヘッダファイル (.h/.hpp) は [`/Core/Inc/`](/Core/Inc/) の中に、ソースファイル (.c/.cpp) は [`/Core/Src/`](/Core/Src/) の中に追加すること。

また、ファイル追加後は以下のように、 [`/CMakeLists.txt`](/CMakeLists.txt) の該当箇所に追加したファイルのパスを追加し、ビルド対象とする。

```cmake
# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/newFile.cpp  # <- here
)
```

ヘッダファイルについては、そのディレクトリを指定する必要があるが `/Core/Src/` については既に通しているので特に必要ない。
