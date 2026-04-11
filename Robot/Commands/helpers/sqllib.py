"""Utility functions for SQLite-backed logging operations.

Only SQLiteFileManager is public. Module-level helpers are intentionally private.
"""

from __future__ import annotations

import sqlite3
from pathlib import Path
from typing import Any, Dict, Iterable


def _sanitize_identifier(name: str) -> str:
    cleaned = "".join(ch if (ch.isalnum() or ch == "_") else "_" for ch in name)
    if not cleaned:
        cleaned = "table"
    if cleaned[0].isdigit():
        cleaned = f"_{cleaned}"
    return cleaned


def _table_name_from_key(filepath: str) -> str:
    return _sanitize_identifier(Path(filepath).stem)


def _db_path_from_key(filepath: str) -> Path:
    key_path = Path(filepath)
    return key_path.parent / "robot_data.db"


def _list_numeric_table_ids(directory: str | Path, include_legacy_csv: bool = True) -> list[int]:
    base_dir = Path(directory)
    ids: list[int] = []

    db_path = base_dir / "robot_data.db"
    if db_path.exists():
        conn = sqlite3.connect(str(db_path))
        try:
            rows = conn.execute("SELECT name FROM sqlite_master WHERE type='table'").fetchall()
            for (table_name,) in rows:
                name = str(table_name)
                if name.startswith("_") and name[1:].isdigit():
                    ids.append(int(name[1:]))
                elif name.isdigit():
                    ids.append(int(name))
        finally:
            conn.close()

    if include_legacy_csv and base_dir.exists():
        for path in base_dir.glob("*.csv"):
            try:
                ids.append(int(path.stem))
            except ValueError:
                pass

    return ids


def _next_numeric_table_key(directory: str | Path, include_legacy_csv: bool = True) -> tuple[int, str]:
    base_dir = Path(directory)
    ids = _list_numeric_table_ids(base_dir, include_legacy_csv=include_legacy_csv)
    next_id = (max(ids) + 1) if ids else 0
    return next_id, str(base_dir / f"{next_id}")


def _ensure_table(conn: sqlite3.Connection, table_name: str, fieldnames: list[str]) -> None:
    cols = [f'"{_sanitize_identifier(field)}" TEXT' for field in fieldnames]
    sql = f'CREATE TABLE IF NOT EXISTS "{table_name}" ({", ".join(cols)})'
    conn.execute(sql)
    conn.commit()


def _setup_table(filepath: str, fieldnames: list[str]) -> tuple[sqlite3.Connection, str, bool]:
    db_path = _db_path_from_key(filepath)
    db_path.parent.mkdir(parents=True, exist_ok=True)
    table_name = _table_name_from_key(filepath)

    conn = sqlite3.connect(str(db_path))
    exists = conn.execute(
        "SELECT 1 FROM sqlite_master WHERE type='table' AND name=?",
        (table_name,),
    ).fetchone()
    _ensure_table(conn, table_name, fieldnames)
    return conn, table_name, exists is None


def _insert_row(conn: sqlite3.Connection, table_name: str, fieldnames: list[str], row: Dict[str, Any]) -> bool:
    columns = [f'"{_sanitize_identifier(field)}"' for field in fieldnames]
    placeholders = ", ".join("?" for _ in fieldnames)
    values = [row.get(field, "") for field in fieldnames]
    sql = f'INSERT INTO "{table_name}" ({", ".join(columns)}) VALUES ({placeholders})'
    conn.execute(sql, values)
    conn.commit()
    return True


def _insert_rows(
    conn: sqlite3.Connection,
    table_name: str,
    fieldnames: list[str],
    rows: Iterable[Dict[str, Any]],
) -> bool:
    columns = [f'"{_sanitize_identifier(field)}"' for field in fieldnames]
    placeholders = ", ".join("?" for _ in fieldnames)
    sql = f'INSERT INTO "{table_name}" ({", ".join(columns)}) VALUES ({placeholders})'
    values = [[row.get(field, "") for field in fieldnames] for row in rows]
    if not values:
        return True
    conn.executemany(sql, values)
    conn.commit()
    return True


class SQLiteFileManager:
    """Manager for multiple SQLite tables with automatic setup and cleanup."""

    def __init__(self):
        self.files: dict[str, tuple[sqlite3.Connection, str, list[str]]] = {}

    def setup_file(self, filepath: str, fieldnames: list[str]) -> tuple[sqlite3.Connection, str]:
        try:
            if filepath not in self.files:
                conn, table_name, _ = _setup_table(filepath, list(fieldnames))
                self.files[filepath] = (conn, table_name, list(fieldnames))
            return self.files[filepath][0], self.files[filepath][1]
        except Exception as e:
            print(f"Error setting up SQLite table for {filepath}: {e}")
            raise

    def write_row(self, filepath: str, row: Dict[str, Any]) -> bool:
        if filepath not in self.files:
            print(f"File key {filepath} not managed. Call setup_file() first.")
            return False
        try:
            conn, table_name, fieldnames = self.files[filepath]
            return _insert_row(conn, table_name, fieldnames, row)
        except Exception as e:
            print(f"Error writing SQLite row for {filepath}: {e}")
            return False

    def write_rows(self, filepath: str, rows: Iterable[Dict[str, Any]]) -> bool:
        if filepath not in self.files:
            print(f"File key {filepath} not managed. Call setup_file() first.")
            return False
        try:
            conn, table_name, fieldnames = self.files[filepath]
            return _insert_rows(conn, table_name, fieldnames, rows)
        except Exception as e:
            print(f"Error bulk-writing SQLite rows for {filepath}: {e}")
            return False

    def overwrite_with_row(self, filepath: str, fieldnames: list[str], row: Dict[str, Any]) -> bool:
        try:
            conn, table_name, _ = _setup_table(filepath, list(fieldnames))
            try:
                conn.execute(f'DELETE FROM "{table_name}"')
                conn.commit()
                return _insert_row(conn, table_name, list(fieldnames), row)
            finally:
                conn.close()
        except Exception as e:
            print(f"Error overwriting SQLite table for {filepath}: {e}")
            return False

    def read_last_row(self, filepath: str) -> dict[str, str] | None:
        try:
            db_path = _db_path_from_key(filepath)
            if not db_path.exists():
                return None

            table_name = _table_name_from_key(filepath)
            conn = sqlite3.connect(str(db_path))
            conn.row_factory = sqlite3.Row
            try:
                exists = conn.execute(
                    "SELECT 1 FROM sqlite_master WHERE type='table' AND name=?",
                    (table_name,),
                ).fetchone()
                if not exists:
                    return None

                row = conn.execute(
                    f'SELECT * FROM "{table_name}" ORDER BY rowid DESC LIMIT 1'
                ).fetchone()
                if row is None:
                    return None
                return {key: row[key] for key in row.keys()}
            finally:
                conn.close()
        except Exception as e:
            print(f"Error reading last SQLite row from {filepath}: {e}")
            return None

    def read_rows(self, filepath: str, limit: int | None = None) -> list[dict[str, str]]:
        try:
            db_path = _db_path_from_key(filepath)
            if not db_path.exists():
                return []

            table_name = _table_name_from_key(filepath)
            conn = sqlite3.connect(str(db_path))
            conn.row_factory = sqlite3.Row
            try:
                exists = conn.execute(
                    "SELECT 1 FROM sqlite_master WHERE type='table' AND name=?",
                    (table_name,),
                ).fetchone()
                if not exists:
                    return []

                sql = f'SELECT * FROM "{table_name}" ORDER BY rowid ASC'
                params: tuple[Any, ...] = ()
                if limit is not None and limit > 0:
                    sql += " LIMIT ?"
                    params = (limit,)

                rows = conn.execute(sql, params).fetchall()
                return [{key: row[key] for key in row.keys()} for row in rows]
            finally:
                conn.close()
        except Exception as e:
            print(f"Error reading SQLite rows from {filepath}: {e}")
            return []

    def next_numeric_table_key(self, directory: str | Path, include_legacy_csv: bool = True) -> tuple[int, str]:
        try:
            return _next_numeric_table_key(directory, include_legacy_csv=include_legacy_csv)
        except Exception as e:
            print(f"Error finding next numeric table key in {directory}: {e}")
            return 0, str(Path(directory) / "0")

    def load_path_points_by_id(self, directory: str | Path, path_id: int) -> list[dict[str, str]]:
        table_key = str(Path(directory) / str(path_id))
        return self.read_rows(table_key)

    def close_all(self):
        for filepath, (conn, _, _) in self.files.items():
            try:
                conn.close()
            except Exception as e:
                print(f"Warning: Failed to close managed db for {filepath}: {e}")
        self.files.clear()

    def close_file(self, filepath: str) -> bool:
        if filepath not in self.files:
            return False
        conn, _, _ = self.files.pop(filepath)
        try:
            conn.close()
            return True
        except Exception as e:
            print(f"Error closing SQLite connection for {filepath}: {e}")
            return False
