import { RENDER_FLOATS_PER_TRIANGLE } from './types';
import type { Bin, BinParameters, BinPiece, Cell } from './types';

const DATABASE_NAME = 'gridfinity-geometry-cache';
const DATABASE_VERSION = 1;
const STORE_NAME = 'meshes';
const LAST_ACCESS_INDEX = 'lastAccess';
const CACHE_KEY_VERSION = 'geometry-cache-v2';
const MAX_CACHE_BYTES = 100 * 1024 * 1024;

interface CachedGeometryRecord {
  key: string;
  pieces: BinPiece[];
  byteSize: number;
  lastAccess: number;
}

let databasePromise: Promise<IDBDatabase> | null = null;

function requestResult<T>(request: IDBRequest<T>): Promise<T> {
  return new Promise((resolve, reject) => {
    request.onsuccess = () => resolve(request.result);
    request.onerror = () => reject(request.error ?? new Error('IndexedDB request failed.'));
  });
}

function transactionComplete(transaction: IDBTransaction): Promise<void> {
  return new Promise((resolve, reject) => {
    transaction.oncomplete = () => resolve();
    transaction.onerror = () => reject(transaction.error ?? new Error('IndexedDB transaction failed.'));
    transaction.onabort = () => reject(transaction.error ?? new Error('IndexedDB transaction aborted.'));
  });
}

function openDatabase(): Promise<IDBDatabase> {
  if (databasePromise) return databasePromise;
  const pending = new Promise<IDBDatabase>((resolve, reject) => {
    const request = indexedDB.open(DATABASE_NAME, DATABASE_VERSION);
    let failed = false;
    const fail = (error: Error) => {
      failed = true;
      reject(error);
    };
    request.onupgradeneeded = () => {
      const store = request.result.createObjectStore(STORE_NAME, { keyPath: 'key' });
      store.createIndex(LAST_ACCESS_INDEX, LAST_ACCESS_INDEX);
    };
    request.onsuccess = () => {
      if (failed) request.result.close();
      else resolve(request.result);
    };
    request.onerror = () => fail(request.error ?? new Error('Unable to open geometry cache.'));
    request.onblocked = () => fail(new Error('Geometry cache upgrade was blocked.'));
  });
  databasePromise = pending;
  void pending.catch(() => {
    if (databasePromise === pending) databasePromise = null;
  });
  return pending;
}

function isCell(value: unknown): value is Cell {
  if (!value || typeof value !== 'object') return false;
  const cell = value as Partial<Cell>;
  return Number.isFinite(cell.x) && Number.isFinite(cell.y);
}

function isPiece(value: unknown): value is BinPiece {
  if (!value || typeof value !== 'object') return false;
  const piece = value as Partial<BinPiece>;
  return piece.vertices instanceof Float32Array &&
    piece.vertices.length > 0 &&
    piece.vertices.length % RENDER_FLOATS_PER_TRIANGLE === 0 &&
    Array.isArray(piece.cells) &&
    piece.cells.every(isCell);
}

function isRecord(value: unknown, key: string): value is CachedGeometryRecord {
  if (!value || typeof value !== 'object') return false;
  const record = value as Partial<CachedGeometryRecord>;
  return record.key === key &&
    Array.isArray(record.pieces) &&
    record.pieces.length > 0 &&
    record.pieces.every(isPiece) &&
    Number.isFinite(record.byteSize) &&
    record.byteSize! > 0 &&
    Number.isFinite(record.lastAccess);
}

function approximateSize(pieces: BinPiece[]): number {
  return pieces.reduce((total, piece) =>
    total + piece.vertices.byteLength + piece.cells.length * 16 + 64, 128);
}

async function removeRecord(key: string): Promise<void> {
  const database = await openDatabase();
  const transaction = database.transaction(STORE_NAME, 'readwrite');
  transaction.objectStore(STORE_NAME).delete(key);
  await transactionComplete(transaction);
}

async function refreshLastAccess(record: CachedGeometryRecord): Promise<void> {
  const database = await openDatabase();
  const transaction = database.transaction(STORE_NAME, 'readwrite');
  record.lastAccess = Date.now();
  transaction.objectStore(STORE_NAME).put(record);
  await transactionComplete(transaction);
}

function cachedBytes(store: IDBObjectStore): Promise<number> {
  return new Promise((resolve, reject) => {
    let totalBytes = 0;
    const request = store.openCursor();
    request.onerror = () => reject(request.error ?? new Error('Unable to scan geometry cache.'));
    request.onsuccess = () => {
      const cursor = request.result;
      if (!cursor) {
        resolve(totalBytes);
        return;
      }
      const record = cursor.value as Partial<CachedGeometryRecord>;
      if (Number.isFinite(record.byteSize) && record.byteSize! > 0) {
        totalBytes += record.byteSize!;
      }
      cursor.continue();
    };
  });
}

function evictOldest(store: IDBObjectStore, initialBytes: number): Promise<void> {
  return new Promise((resolve, reject) => {
    let totalBytes = initialBytes;
    const request = store.index(LAST_ACCESS_INDEX).openCursor();
    request.onerror = () => reject(request.error ?? new Error('Unable to evict geometry cache.'));
    request.onsuccess = () => {
      const cursor = request.result;
      if (!cursor || totalBytes <= MAX_CACHE_BYTES) {
        resolve();
        return;
      }
      const record = cursor.value as Partial<CachedGeometryRecord>;
      cursor.delete();
      if (Number.isFinite(record.byteSize) && record.byteSize! > 0) {
        totalBytes -= record.byteSize!;
      }
      cursor.continue();
    };
  });
}

async function evictLeastRecentlyUsed(): Promise<void> {
  const database = await openDatabase();
  const sizeTransaction = database.transaction(STORE_NAME, 'readonly');
  const sizeComplete = transactionComplete(sizeTransaction);
  const totalBytes = await cachedBytes(sizeTransaction.objectStore(STORE_NAME));
  await sizeComplete;
  if (totalBytes <= MAX_CACHE_BYTES) return;

  const evictionTransaction = database.transaction(STORE_NAME, 'readwrite');
  const evictionComplete = transactionComplete(evictionTransaction);
  await evictOldest(evictionTransaction.objectStore(STORE_NAME), totalBytes);
  await evictionComplete;
}

/** Hash only worker-consumed geometry parameters, excluding editor-owned bin identity. */
export async function geometryCacheKey(parameters: BinParameters): Promise<string> {
  const geometryParameters = {
    height: parameters.height,
    perimeterThickness: parameters.perimeterThickness,
    filletRadius: parameters.filletRadius,
    fasteners: parameters.fasteners,
    cells: parameters.cells,
    openings: parameters.openings,
    walls: parameters.walls,
    pieces: parameters.pieces,
  };
  const bytes = new TextEncoder().encode(`${CACHE_KEY_VERSION}:${JSON.stringify(geometryParameters)}`);
  const digest = await crypto.subtle.digest('SHA-256', bytes);
  return Array.from(new Uint8Array(digest), (byte) => byte.toString(16).padStart(2, '0')).join('');
}

export async function readCachedBin(key: string, binId: string): Promise<Bin | null> {
  try {
    const database = await openDatabase();
    const transaction = database.transaction(STORE_NAME, 'readonly');
    const store = transaction.objectStore(STORE_NAME);
    const value = await requestResult(store.get(key));
    if (!isRecord(value, key)) {
      if (value !== undefined) void removeRecord(key).catch(() => undefined);
      return null;
    }
    void refreshLastAccess(value).catch(() => undefined);
    return { binId, pieces: value.pieces };
  } catch {
    return null;
  }
}

export async function writeCachedBin(key: string, bin: Bin): Promise<void> {
  const record: CachedGeometryRecord = {
    key,
    pieces: bin.pieces,
    byteSize: approximateSize(bin.pieces),
    lastAccess: Date.now(),
  };
  const database = await openDatabase();
  const transaction = database.transaction(STORE_NAME, 'readwrite');
  transaction.objectStore(STORE_NAME).put(record);
  await transactionComplete(transaction);
  await evictLeastRecentlyUsed();
}
