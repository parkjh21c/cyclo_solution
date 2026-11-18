# 커밋 히스토리 페이지

브랜치별 git 로그를 읽어 **어떤 기능이 언제, 어떻게 바뀌었는지** 한 페이지로 보여 주는 정적 사이트를 만듭니다.
`.github/workflows/commit-history.yml`이 `demo-1f`에 push될 때마다 페이지를 다시 만들어 GitHub Pages에 배포합니다.

## 페이지 구성

- **기능 지도**: 행은 기능 영역, 열은 커밋(왼쪽이 오래된 것)입니다. 점 크기는 그 커밋이 해당 영역에서 바꾼 줄 수입니다.
  열을 누르면 그 커밋이 펼쳐지고, 영역 이름을 누르면 그 영역만 걸러 봅니다.
- **기능별 변화**: 영역마다 커밋 수, 변경 줄 수, 커밋 분포, 마지막 변경을 보여 줍니다.
- **커밋 목록**: 날짜별로 묶은 커밋입니다. 펼치면 본문 설명, `Validation:` 문단(검증), 변경 파일과 영역,
  GitHub diff 링크가 나옵니다. 검색(제목·본문·파일 경로·SHA)과 변경 종류(추가·수정·개선·제거 등) 필터가 있습니다.
- 브랜치 탭(`demo-1f`, `main`, `jazzy`), 밝은/어두운 모드, URL 해시(`#b=demo-1f&a=cup&c=0868d6f`)로 화면 공유가 됩니다.

## 로컬에서 만들기

표준 라이브러리만 씁니다.

```bash
git fetch origin '+refs/heads/*:refs/remotes/origin/*'
python3 tools/commit_history/build_history.py --out _site --default-branch demo-1f
xdg-open _site/index.html
```

옵션: `--branch <이름>`(반복 가능, 기본은 origin의 모든 브랜치), `--max-commits N`, `--repo-url`, `--title`.

## 기능 영역 규칙

`areas.json`에서 정합니다. 바뀐 파일마다 **위에서부터 처음 맞는 패턴**의 영역에 들어갑니다
(fnmatch, 대소문자 무시, `*`는 `/`도 포함). 커밋의 영역은 파일 영역들의 합이고, 줄 수가 가장 많은 영역이 먼저 표시됩니다.
어느 패턴에도 맞지 않으면 `기타`입니다. 새 모듈을 추가하면 여기에 패턴을 넣습니다.

변경 종류는 커밋 제목의 첫 단어로 정합니다(`Add` → 추가, `Fix`/`failing` → 수정, `Remove`/`Drop` → 제거,
`Refactor` → 리팩터, 문서 파일만 바뀌면 문서, lint·CI·Docker → 유지보수, 그 밖은 개선).

## 배포 설정 (한 번만)

1. 저장소 **Settings → Pages → Build and deployment → Source**를 **GitHub Actions**로 바꿉니다.
2. **Settings → Environments → github-pages → Deployment branches and tags**에 `demo-1f`를 추가합니다.
   기본값은 기본 브랜치(`main`)만 배포를 허용합니다.
3. **Actions → Commit history page → Run workflow**로 처음 배포하거나 `demo-1f`에 push합니다.

페이지에는 커밋 제목·본문·작성자 이름·파일 경로가 공개됩니다(이메일은 넣지 않습니다).
