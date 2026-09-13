#!/usr/bin/env bash
# ajouatom/openpilot carrot-wip 최신 변경을 받아 내 브랜치에 merge 합니다.
# merge 방식이라 포크에 강제 push가 필요 없고, 기기의 자동 업데이트/git pull
# (git reset --hard 후 fast-forward 병합)이 그대로 동작합니다.
# upstream(ajouatom)으로는 절대 push 하지 않습니다 (push URL 비활성화됨).
#
# 사용법: ./sync-upstream.sh [브랜치명]   (기본값: my-hud)
set -euo pipefail
cd "$(dirname "$0")"

BRANCH="${1:-my-hud}"

if [ -n "$(git status --porcelain --untracked-files=no)" ]; then
  echo "커밋되지 않은 변경이 있습니다. 먼저 커밋하거나 'git stash' 하세요." >&2
  exit 1
fi

git switch "$BRANCH"
git fetch upstream carrot-wip

# merge 전 상태를 백업 (문제가 생기면: git reset --hard ${BRANCH}-backup)
git branch -f "${BRANCH}-backup" "$BRANCH"

if ! git merge --no-edit upstream/carrot-wip; then
  echo
  echo "충돌이 발생했습니다. 파일을 수정한 뒤:"
  echo "  git add <파일> && git commit --no-edit"
  echo "포기하고 원래대로 돌리려면:"
  echo "  git merge --abort"
  exit 1
fi

echo
echo "완료: upstream/carrot-wip ($(git rev-parse --short upstream/carrot-wip)) 를 $BRANCH 에 merge 했습니다."
echo "원본 대비 내 변경 파일:"
git diff --stat upstream/carrot-wip "$BRANCH"

if git remote get-url origin >/dev/null 2>&1; then
  echo
  echo "포크에 반영하려면: git push origin $BRANCH"
fi
