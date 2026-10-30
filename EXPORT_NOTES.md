# 源码导出说明（2026-10-30）

这是当前工作区的源码快照，包含尚未提交的修改及新增源文件。ZIP 内不含 .git 历史、
build/install/log、运行验证目录、录包、高程数据库、生成地图和本机 .deps。
原仓库已跟踪的模型、网格、测试地图和第三方资源保留，以免破坏构建及测试。
本包是源码，需要按项目文档重新安装依赖、构建后使用。

## 上传 GitHub

先解压，再把源码提交到对应 Git 仓库；不要把 ZIP 本身当作源码目录。
更新已有仓库时保留其 .git，不要用此快照覆盖或删除历史。
Autoware 和 ELE Scout 保持两个独立仓库。

## ELE Scout 的最新修改

最新射线清除修复的一部分在独立 ELE Scout 中。配套的 ele_scouts_kit_source ZIP
包含完整当前源码和 HT 模型。也可在干净的 ELE 基线提交上应用本包 patches 中的补丁。
repositories/scout-external.repos 仍记录原来的基线提交；仅 vcs import 不能取得这些未提交修改。
补丁已验证可应用且复现当前 ELE 的全部源文件。上传 ELE 新提交后，
将该 repos 文件的 version 更新为实际上传的提交 SHA；本次打包不假设新提交已存在。

例如导入基线后，从 Autoware 根目录执行：

```bash
git -C ../scout_dependencies/ele_scouts_kit apply \
  "$PWD/patches/ele_scouts_kit_local_20261030.patch"
```

若使用配套 ELE ZIP，则它已经包含这些修改，不要再重复应用补丁。
构建 ELE 后可用 SCOUT_HT_ELE_WORKSPACE 指定解压后的工作区路径。

## 校验和与验证范围

MANIFEST_SHA256.txt 记录文件内容校验和。SOURCE_SNAPSHOT.json 记录基线和快照信息。
最新射线修复通过 66 项 ELE/CUDA 测试和 93 项 Scout 回归，共 159 项。
现场走人复测主增高区域 69 格在离开后恢复；5 个零散格仍有当前邻近回波，需要继续核对输入。
这不是“所有现场高点已解决”或“自动驾驶已完成实车验收”的声明。
原文档中 validation/ 链接对应本机实验记录，本源码包未携带这些传感器数据和运行报告。
