# 线路资源导出

`orvd_export_track_bundle` 直接采样线路，生成 `orvd.scene_bundle` 资源目录。

```sh
build/tools/scene_export/orvd_export_track_bundle \
  --track track_library/geometries/straight_level_1100m.json \
  --start 0 --end 1100 --spacing 0.5 --output /tmp/track-bundle \
  --rail-datums-from /path/to/record/scene.json
```

`--rail-datums-from` 接受记录或资源包的 `scene.json`，从首站反算钢轨基准。
也可改用四个显式参数：`--left-lateral`、`--left-vertical`、
`--right-lateral`、`--right-vertical`；单位为米，线路系向右、向下为正。
输出保存实际使用的四个基准值。

可用 `--visual-definition PATH` 附带视觉定义。采样包含精确终点，输出目录须不存在。
工具不安装；生成的资源包可通过查看器的 `bundle=` 参数加载。
