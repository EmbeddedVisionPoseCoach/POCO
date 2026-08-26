from queue import Empty, Full


def put_latest(queue, data):
    """상태(State) Queue에 최신 데이터 하나만 유지한다.

    실시간 landmark / sensor / process state처럼 과거 값이 필요 없는 데이터에만 사용한다.
    Command / Event에는 사용하지 않는다.
    """
    try:
        queue.put_nowait(data)
        return True
    except Full:
        pass

    try:
        queue.get_nowait()
    except Empty:
        pass

    try:
        queue.put_nowait(data)
        return True
    except Full:
        return False


def get_latest(queue, default=None):
    """상태 Queue를 비우며 가장 최신 항목만 반환한다."""
    latest = default

    while True:
        try:
            latest = queue.get_nowait()
        except Empty:
            return latest


def put_ordered(queue, data, timeout=0.2):
    """Command / Event를 순서를 유지해 넣는다.

    기존 항목을 임의로 버리지 않는다. 큐가 가득 찬 경우 timeout 동안 기다린 뒤 False를 반환한다.
    """
    try:
        queue.put(data, timeout=timeout)
        return True
    except Full:
        return False


def drain_ordered(queue):
    """현재 도착해 있는 Command / Event를 순서대로 모두 꺼낸다."""
    items = []

    while True:
        try:
            items.append(queue.get_nowait())
        except Empty:
            return items
