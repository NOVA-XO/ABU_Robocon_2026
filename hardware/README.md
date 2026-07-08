# Hardware — PCB (EasyEDA)

Хоёр робот (**Robot1** гар, **Robot2** автомат) **нэг ижил PCB** дээр
угсрагдсан. PCB-г EasyEDA дээр зурсан.

## Файлаа энд нэмнэ

EasyEDA-аас export хийж дараах файлуудыг **энэ фолдерт** хий:

| Файл | Үүсгэх арга (EasyEDA) | Зорилго |
|------|----------------------|---------|
| **`pcb.png`** | `Export → PNG` (эсвэл schematic-ийн зураг) | README-д харагдана (**зайлшгүй**) |
| `gerber.zip` | `Fabrication → Gerber` | Үйлдвэрлэлд (сонголт) |
| `schematic.pdf` | `Document → Export → PDF` | Холболтын баримт (сонголт) |
| EasyEDA source `.json` | `Document → Export → EasyEDA Source` | Дараа засварлахад (сонголт) |

## Тэмдэглэл

- `pcb.png`-г энэ нэрээр хийвэл root [../README.md](../README.md)-ийн зураг
  автоматаар харагдана.
- Хэд хэдэн зураг байвал (схем + PCB) `schematic.png`, `pcb.png` гэх мэтээр
  нэрлээд README-д нэмж болно.
