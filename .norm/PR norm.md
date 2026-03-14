# PR 规范
> PR：Pull Request，用于将开发分支代码合并到 main

## 1 分支开发

每个人使用独立分支开发，例如：

feature/wrj
feature/zc

禁止直接在 main 分支开发。

---

## 2 同步 main 分支

开发过程中应定期同步 main：

git fetch origin
git rebase origin/main

避免使用 git merge origin/main。

---

## 3 提交 PR

当功能开发完成并准备合入主分支时：

1 push 分支

git push origin feature/xxx

2 创建 PR

feature/xxx → main

---

## 4 PR 冲突处理

若 PR 出现冲突：

开发者需要同步 main 并解决冲突：

git fetch origin
git rebase origin/main

解决冲突后：

git push -f

更新 PR。

---

## 5 PR 合并

PR 审核通过后，由仓库维护者合并到 main。

推荐使用 Squash Merge 保持提交历史整洁。