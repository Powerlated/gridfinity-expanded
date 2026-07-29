use bytemuck::Pod;
use std::num::NonZeroU64;

const ALIGNMENT: u64 = 256;
pub const SLOTS: u64 = 1024;

pub struct Arena {
    buffer: wgpu::Buffer,
    cursor: u64,
    stride: u64,
    capacity: u64,
}

impl Arena {
    pub fn new(device: &wgpu::Device, label: &str, size: usize) -> Arena {
        let stride = (size as u64).div_ceil(ALIGNMENT) * ALIGNMENT;
        let capacity = stride * SLOTS;
        Arena {
            buffer: device.create_buffer(&wgpu::BufferDescriptor {
                label: Some(label),
                size: capacity,
                usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            }),
            cursor: 0,
            stride,
            capacity,
        }
    }

    pub fn reset(&mut self) {
        self.cursor = 0;
    }

    pub fn push<T: Pod>(&mut self, queue: &wgpu::Queue, value: &T) -> u64 {
        let offset = if self.cursor + self.stride > self.capacity { 0 } else { self.cursor };
        queue.write_buffer(&self.buffer, offset, bytemuck::bytes_of(value));
        self.cursor = offset + self.stride;
        offset
    }

    pub fn binding(&self, offset: u64, size: usize) -> wgpu::BindingResource<'_> {
        wgpu::BindingResource::Buffer(wgpu::BufferBinding {
            buffer: &self.buffer,
            offset,
            size: NonZeroU64::new(size as u64),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_frame_gets_far_more_slots_than_the_pass_chain_can_ask_for() {
        assert!(SLOTS > 64, "the deepest frame issues a few dozen draws");
    }

    #[test]
    fn an_offset_always_lands_on_a_uniform_binding_boundary() {
        for size in [96usize, 208, 240, 260] {
            let stride = (size as u64).div_ceil(ALIGNMENT) * ALIGNMENT;
            assert_eq!(stride % ALIGNMENT, 0);
            assert!(stride >= size as u64);
        }
    }
}
