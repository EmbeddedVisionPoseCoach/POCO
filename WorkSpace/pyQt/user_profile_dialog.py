"""Compact four-slot profile chooser used by the 7-inch UI."""

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QDialog, QGridLayout, QHBoxLayout, QLabel, QLineEdit, QMessageBox,
    QPushButton, QVBoxLayout,
)


class UserProfileDialog(QDialog):
    def __init__(self, profile_service, save_mode=False, parent=None):
        super().__init__(parent)
        self.profile_service = profile_service
        self.save_mode = bool(save_mode)
        self.selected_slot = None
        self.selected_name = ""
        self.setWindowTitle("보정 프로필 저장" if save_mode else "사용자 프로필 선택")
        self.setFixedSize(620, 360)
        self._build_ui()
        self.refresh()

    def _build_ui(self):
        outer = QVBoxLayout(self)
        title = QLabel(
            "저장할 슬롯과 이름을 선택하세요."
            if self.save_mode else
            "프로필을 선택한 뒤 아래 버튼을 누르세요. 측정은 메인 화면에서 별도로 시작합니다."
        )
        title.setWordWrap(True)
        outer.addWidget(title)
        self.grid = QGridLayout()
        self.slot_buttons = []
        self.delete_buttons = []
        for row in range(4):
            slot = row + 1
            button = QPushButton()
            button.setCheckable(True)
            button.setMinimumHeight(48)
            button.clicked.connect(lambda _checked=False, value=slot: self._select(value))
            delete = QPushButton("삭제")
            delete.setFixedWidth(70)
            delete.clicked.connect(lambda _checked=False, value=slot: self._delete(value))
            self.grid.addWidget(button, row, 0)
            self.grid.addWidget(delete, row, 1)
            self.slot_buttons.append(button)
            self.delete_buttons.append(delete)
        outer.addLayout(self.grid)
        if self.save_mode:
            name_row = QHBoxLayout()
            name_row.addWidget(QLabel("프로필 이름"))
            self.name_edit = QLineEdit()
            self.name_edit.setMaxLength(24)
            self.name_edit.setPlaceholderText("예: 홍길동")
            name_row.addWidget(self.name_edit)
            outer.addLayout(name_row)
        bottom = QHBoxLayout()
        bottom.addStretch(1)
        confirm = QPushButton("프로필 저장" if self.save_mode else "선택한 프로필 불러오기")
        confirm.setMinimumHeight(38)
        confirm.clicked.connect(self._confirm)
        cancel = QPushButton("취소")
        cancel.clicked.connect(self.reject)
        bottom.addWidget(confirm)
        bottom.addWidget(cancel)
        outer.addLayout(bottom)

    def refresh(self):
        profiles = self.profile_service.list_profiles()
        for profile, button, delete in zip(profiles, self.slot_buttons, self.delete_buttons):
            slot = profile["slot"]
            occupied = profile.get("occupied", False)
            name = profile.get("name", "빈 슬롯")
            button.setText(f"슬롯 {slot}  |  {name}")
            button.setChecked(slot == self.selected_slot)
            delete.setEnabled(occupied)

    def _select(self, slot):
        self.selected_slot = int(slot)
        for index, button in enumerate(self.slot_buttons, start=1):
            button.setChecked(index == self.selected_slot)
        if self.save_mode:
            profile = self.profile_service.list_profiles()[slot - 1]
            if profile.get("occupied") and not self.name_edit.text().strip():
                self.name_edit.setText(str(profile.get("name", "")))

    def _delete(self, slot):
        answer = QMessageBox.question(
            self, "프로필 삭제", f"슬롯 {slot}의 프로필을 삭제할까요?",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No,
        )
        if answer == QMessageBox.Yes:
            self.profile_service.delete_profile(slot)
            if self.selected_slot == slot:
                self.selected_slot = None
            self.refresh()

    def _confirm(self):
        if self.selected_slot is None:
            QMessageBox.warning(self, "프로필", "슬롯을 먼저 선택해주세요.")
            return
        profile = self.profile_service.list_profiles()[self.selected_slot - 1]
        if self.save_mode:
            name = self.name_edit.text().strip()
            if not name:
                QMessageBox.warning(self, "프로필", "프로필 이름을 입력해주세요.")
                return
            if profile.get("occupied"):
                answer = QMessageBox.question(
                    self, "덮어쓰기 확인", f"슬롯 {self.selected_slot}을 덮어쓸까요?",
                    QMessageBox.Yes | QMessageBox.No, QMessageBox.No,
                )
                if answer != QMessageBox.Yes:
                    return
            self.selected_name = name
        elif not profile.get("occupied"):
            QMessageBox.warning(self, "프로필", "빈 슬롯은 불러올 수 없습니다.")
            return
        self.accept()
