# 个人作品集网站

轻量级静态网站，适合 2核2G 服务器部署。

## 目录结构

```
MyWeb/
├── index.html          # 首页
├── project-1.html      # 项目详情页示例
├── css/
│   ├── style.css       # 主样式
│   └── project.css     # 项目详情页样式
├── js/
│   └── main.js         # 交互脚本
├── images/             # 存放你的图片
└── projects/           # 存放更多项目页面
```

## 如何使用

### 1. 本地预览

```bash
# 进入项目目录
cd /root/IOT/MyWeb

# 启动简单 HTTP 服务器
python3 -m http.server 8080

# 或使用 Node.js
npx serve .
```

然后访问 http://localhost:8080

### 2. 修改内容

**修改个人信息：**
- 编辑 `index.html` 中的文字
- 替换占位图片为你的项目图片

**添加新项目：**
1. 复制 `project-1.html` 为 `project-2.html`
2. 修改内容
3. 在 `index.html` 的项目卡片中添加链接

### 3. 部署到服务器

```bash
# 安装 Nginx
apt update && apt install nginx -y

# 复制文件到网站目录
cp -r /root/IOT/MyWeb/* /var/www/html/

# 启动 Nginx
systemctl start nginx
```

### 4. 配置域名和 HTTPS

```bash
# 安装 Certbot
apt install certbot python3-certbot-nginx -y

# 获取 SSL 证书
certbot --nginx -d yourdomain.com
```

## 自定义

### 修改颜色

编辑 `css/style.css` 中的 CSS 变量：

```css
:root {
    --primary: #2563eb;      /* 主色调 */
    --primary-dark: #1d4ed8; /* 深色主色调 */
    --text: #1e293b;         /* 文字颜色 */
}
```

### 添加视频

支持嵌入 B站/YouTube 视频：

```html
<!-- YouTube -->
<div class="video-container">
    <iframe src="https://www.youtube.com/embed/视频ID" ...></iframe>
</div>

<!-- B站 -->
<div class="video-container">
    <iframe src="//player.bilibili.com/player.html?bvid=BV..." ...></iframe>
</div>
```

## 特性

- ✅ 响应式设计，支持移动端
- ✅ 平滑滚动动画
- ✅ 图片懒加载
- ✅ 支持视频嵌入
- ✅ 极低资源占用
- ✅ 易于自定义
