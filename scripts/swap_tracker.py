#!/usr/bin/env python3
"""추적기를 「군집 채널을 켠 것」과 「원본」 사이에서 바꿔 끼운다 (WORKLOG 38).

    swap_tracker.py clustering   # 군집 채널을 켜서 계획이 그것을 쓰게 한다
    swap_tracker.py original     # 원래대로 되돌린다

원본 커맨드라인을 그대로 재생하고 두 군데만 바꾼다 —
`detection02` 리맵을 군집 출력으로, 그 채널을 `lidar_clustering` 로.
**노드 이름·네임스페이스·출력 토픽은 그대로**라서 하류(계획)는 아무것도 모른다.
이름이 같으니 원본 파라미터 파일도 키가 그대로 맞는다.

커맨드라인은 처음 한 번 저장해 두고 복구에 쓴다. 저장 파일의 `--params-file` 중
`/tmp/launch_params_*` 는 런치가 만든 임시 파일이라 **스택을 재기동하면 무효**가 된다.
그때는 이 스크립트를 쓰지 말고 스택을 다시 띄우면 원래 구성으로 돌아온다.
"""

import os
import subprocess
import sys
import time

SAVE = "/tmp/shadow_tracker/tracker_argv_original.txt"
LOG = "/tmp/shadow_tracker/tracker.log"
CLUSTER_TOPIC = "/perception/object_recognition/detection/clustering/objects"
SLOT = "~/input/detection02/objects"
NODE_KEY = "/perception/object_recognition/tracking/multi_object_tracker"
OVERRIDE = "/tmp/shadow_tracker/channel_override.param.yaml"


def running_pid():
    out = subprocess.run(["ps", "-eo", "pid,comm"], capture_output=True, text=True).stdout
    for line in out.splitlines():
        p = line.split()
        if len(p) == 2 and p[1].startswith("multi_object_tr"):
            return int(p[0])
    return None


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else ""
    if mode not in ("clustering", "original"):
        raise SystemExit("사용법: swap_tracker.py {clustering|original}")

    os.makedirs("/tmp/shadow_tracker", exist_ok=True)
    pid = running_pid()
    if pid and not os.path.exists(SAVE):
        with open(f"/proc/{pid}/cmdline", "rb") as f:
            argv = [a for a in f.read().decode().split("\0") if a]
        open(SAVE, "w").write("\n".join(argv))
        print(f"원본 커맨드라인 저장 ({len(argv)} 인자) → {SAVE}")
    if not os.path.exists(SAVE):
        raise SystemExit("원본 커맨드라인이 없다 — 스택이 떠 있을 때 한 번 실행해야 한다")

    argv = open(SAVE).read().split("\n")
    if mode == "clustering":
        argv = [f"{SLOT}:={CLUSTER_TOPIC}" if a.startswith(f"{SLOT}:=") else a for a in argv]
        # 채널은 `-p` 로 못 바꾼다. 노드 이름으로 키가 걸린 `--params-file` 항목이
        # 뒤에 붙인 `-p`(와일드카드 급)를 이긴다 — 실측: -p 를 줘도 값이 none 이었다.
        # 같은 급(노드 이름 키)의 파일을 하나 더, **맨 뒤에** 붙여 이긴다.
        with open(OVERRIDE, "w") as f:
            f.write(f"{NODE_KEY}:\n  ros__parameters:\n"
                    f"    input/detection02/channel: lidar_clustering\n")
        argv += ["--params-file", OVERRIDE]

    if pid:
        os.kill(pid, 15)
        for _ in range(50):
            time.sleep(0.2)
            if running_pid() is None:
                break
        print(f"기존 추적기 종료 (pid {pid})")

    log = open(LOG, "a")
    subprocess.Popen(argv, stdout=log, stderr=log, start_new_session=True)
    time.sleep(6)
    new = running_pid()
    print(f"{mode} 추적기 기동 (pid {new}) — 로그 {LOG}")
    if new is None:
        raise SystemExit("기동 실패 — 로그를 볼 것")


if __name__ == "__main__":
    main()
