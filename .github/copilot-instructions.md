<!-- mermaid-ai-skills:start -->
## Mermaid Diagrams

When the user asks to create, edit, or visualize a diagram, follow the
instructions in `.github/instructions/mermaid.instructions.md`.
<!-- mermaid-ai-skills:end -->

## C++ 代码风格（强制）

适用：手写 C++ 源文件（`ApplicationCpp/**`、`checks/**`）。
不适用：CubeMX 生成代码与第三方库（`Core/`、`Drivers/`、`USB_DEVICE/`、`Middlewares/`）。

**大括号必须单起一行（Allman 风格）**：`if` / `else` / `for` / `while` /
`switch` / `do` 以及函数/类定义的左大括号 `{` 另起一行，并与语句关键字
**左对齐**；右大括号 `}` 同样单独一行。

禁止：

```cpp
if (status != PluginStatus::Ok) {
    return status;
}
```

必须：

```cpp
if (status != PluginStatus::Ok)
{
    return status;
}
```

详情见仓库根目录 `AGENTS.md`。
