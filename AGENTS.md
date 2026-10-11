# AGENTS.md —— 本仓库 AI 编码助手的通用约定

本文件对所有 AI 编码助手（Codex / GitHub Copilot / Claude Code 等）生效。
**修改本仓库代码前必须遵守。**

## 适用范围

- 适用范围：**手写的 C++ 源文件**，即 `ApplicationCpp/**`、`checks/**`。
- **不适用**：CubeMX 生成代码与第三方库（`Core/`、`Drivers/`、`USB_DEVICE/`、
  `Middlewares/` 等）。这些文件**禁止改动**，除非改动严格落在
  `/* USER CODE BEGIN xxx */ ... /* USER CODE END xxx */` 段内。

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

`else` / `else if` 也另起一行：

```cpp
if (a)
{
    doA();
}
else if (b)
{
    doB();
}
else
{
    doC();
}
```

`for` / `while` 同理：

```cpp
for (uint16_t i = 0U; i < count; ++i)
{
    process(i);
}

while (running)
{
    step();
}
```

> 例外：单行内联的短函数体（例如访问器 `PluginId id() const { return id_; }`）
> 可保持原样。**新增或修改的代码一律使用上述风格。**
