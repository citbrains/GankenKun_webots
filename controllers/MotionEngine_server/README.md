# Build requirement
- libzmq https://zeromq.org/download/
- cppzmq https://github.com/zeromq/cppzmq

## windowsでネイティブに動くwebotsをwslから動かす方法
wsl2上で動かしている歩行制御器とwindows側のネイティブなwebotsで通信を行い、シミュレーションを行う事ができる。wsl2のlocalhostforwadingを利用して実現している。
1. https://qiita.com/kkoba775/items/d28b30a6e1f13803036f を参考にしてwebotsのdllをビルドする。
2. vcpkgを使ってprotobufとzeromqとboostをインストールする。
3. Visual Studioで、コントローラのCMakeListsを使ってビルドする。CMakeのconfigureの時にはDVCPKG_TARGET_TRIPLETとDCMAKE_TOOLCHAIN_FILEをパラメータとして渡す。
4. hr46_b3m側ではwindows向けのビルド設定でビルドする。
5. wsl2のlocalhostforwadingを有効にする。