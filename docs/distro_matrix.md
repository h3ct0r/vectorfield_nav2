# Distro matrix

| ROS 2 distro | Ubuntu | Nav2 controller API | Status |
|---|---|---|---|
| Humble | 22.04 (jammy) | Gen A/B: `setPlan`; the plugin's own `LegacyPathHandler` | Supported; gating CI |
| Jazzy | 24.04 (noble) | Gen A/B: `setPlan`; the plugin's own `LegacyPathHandler` | Supported; gating CI on amd64 and arm64 |
| Kilted | 24.04 (noble) | Gen A/B | Unsupported: shares Jazzy's API and may build, but has no CI, configuration or release |
| Lyrical | 26.04 (resolute) | Gen C: `newPathReceived`; the server's path handler | Supported; gating CI |
| Rolling | 26.04 (resolute) | Gen C | Built and tested nightly, without launch tests; failures do not gate |

:::{note}
Binary packages, the platforms the ROS build farm builds them for, and the per-distro
differences in demos and launch files are planned for M6, with the 0.1.0 release.
:::
