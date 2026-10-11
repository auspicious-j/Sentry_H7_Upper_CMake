# CLAUDE.md

本仓库对 AI 编码助手的约定，详见仓库根目录的 [`AGENTS.md`](./AGENTS.md)。
（内容与 AGENTS.md 保持一致，避免不同 agent 读到不一致的规则。）

## 强制规则：C++ 大括号单起一行（Allman 风格）

所有控制语句（`if` / `else` / `for` / `while` / `switch` / `do`）以及
函数、类、命名空间定义的左大括号 `{`，**必须另起一行**，并与语句关键字
**左对齐**；右大括号 `}` 同样单独一行。

**禁止（K&R 风格）：**

```cpp
if (status != PluginStatus::Ok) {
    return status;
}
```

**必须：**

```cpp
if (status != PluginStatus::Ok)
{
    return status;
}
```

`else` / `else if` / `for` / `while` 同样处理。

## 适用范围

- 适用于**手写的 C++ 源文件**：`ApplicationCpp/**`、`checks/**`。
- **不适用**于 CubeMX 生成代码与第三方库（`Core/`、`Drivers/`、`USB_DEVICE/`、
  `Middlewares/`）；这些文件禁止改动，除非改动落在 `/* USER CODE BEGIN */` 段内。
