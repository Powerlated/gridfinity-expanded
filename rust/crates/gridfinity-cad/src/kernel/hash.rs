use std::hash::{BuildHasherDefault, Hasher};

const SEED: u64 = 0x51_7c_c1_b7_27_22_0a_95;

#[derive(Default, Clone, Copy)]
pub struct FxHasher {
    hash: u64,
}

impl FxHasher {
    #[inline]
    fn add(&mut self, i: u64) {
        self.hash = (self.hash.rotate_left(5) ^ i).wrapping_mul(SEED);
    }
}

impl Hasher for FxHasher {
    #[inline]
    fn finish(&self) -> u64 {
        self.hash
    }

    #[inline]
    fn write(&mut self, bytes: &[u8]) {
        let mut rest = bytes;
        while rest.len() >= 8 {
            self.add(u64::from_ne_bytes(rest[..8].try_into().unwrap()));
            rest = &rest[8..];
        }
        if rest.len() >= 4 {
            self.add(u32::from_ne_bytes(rest[..4].try_into().unwrap()) as u64);
            rest = &rest[4..];
        }
        for &b in rest {
            self.add(b as u64);
        }
    }

    #[inline]
    fn write_u8(&mut self, i: u8) {
        self.add(i as u64);
    }
    #[inline]
    fn write_u32(&mut self, i: u32) {
        self.add(i as u64);
    }
    #[inline]
    fn write_u64(&mut self, i: u64) {
        self.add(i);
    }
    #[inline]
    fn write_usize(&mut self, i: usize) {
        self.add(i as u64);
    }
    #[inline]
    fn write_i64(&mut self, i: i64) {
        self.add(i as u64);
    }
    #[inline]
    fn write_i32(&mut self, i: i32) {
        self.add(i as u64);
    }
}

pub type FxBuildHasher = BuildHasherDefault<FxHasher>;
pub type FxHashMap<K, V> = std::collections::HashMap<K, V, FxBuildHasher>;
pub type FxHashSet<K> = std::collections::HashSet<K, FxBuildHasher>;

#[cfg(test)]
mod tests {
    use super::*;
    use std::hash::Hash;

    fn h<T: Hash>(v: &T) -> u64 {
        let mut s = FxHasher::default();
        v.hash(&mut s);
        s.finish()
    }

    #[test]
    fn equal_keys_hash_equal_and_distinct_keys_mostly_differ() {
        let a = (1i64, 2i64, 3i64);
        assert_eq!(h(&a), h(&(1i64, 2i64, 3i64)));
        assert_ne!(h(&a), h(&(3i64, 2i64, 1i64)));
    }

    /// The weld keys this hashes are small dense integer triples; a hasher that
    /// collapses those would silently turn interning into a linear scan.
    #[test]
    fn dense_integer_triples_spread_across_buckets() {
        let mut seen = FxHashSet::default();
        for x in 0..40i64 {
            for y in 0..40i64 {
                for z in 0..10i64 {
                    seen.insert(h(&(x, y, z)));
                }
            }
        }
        assert_eq!(seen.len(), 40 * 40 * 10, "hash collisions on dense triples");
    }

    #[test]
    fn map_round_trips() {
        let mut m: FxHashMap<(i64, i64, i64), usize> = FxHashMap::default();
        for i in 0..1000i64 {
            m.insert((i, -i, i * 2), i as usize);
        }
        assert_eq!(m.len(), 1000);
        for i in 0..1000i64 {
            assert_eq!(m.get(&(i, -i, i * 2)), Some(&(i as usize)));
        }
    }
}
