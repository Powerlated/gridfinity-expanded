use flate2::Compression;
use flate2::write::GzEncoder;
use gridfinity_cad::badapple::FRAME_BYTES;
use std::io::Write;
use std::path::PathBuf;

fn main() {
    let mut args = std::env::args().skip(1);
    let input = PathBuf::from(args.next().unwrap_or_else(|| {
        eprintln!("usage: compress_badapple <badapple.raw> [out.gz]");
        std::process::exit(2);
    }));
    let output = args.next().map(PathBuf::from).unwrap_or_else(|| {
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("assets/badapple.raw.gz")
    });

    let raw = std::fs::read(&input).expect("cannot read the raw frame dump");
    assert!(
        raw.len() % FRAME_BYTES == 0,
        "{} is {} bytes, not a whole number of {FRAME_BYTES}-byte frames",
        input.display(),
        raw.len()
    );

    let mut encoder = GzEncoder::new(Vec::new(), Compression::best());
    encoder.write_all(&raw).expect("gzip write");
    let packed = encoder.finish().expect("gzip finish");

    std::fs::create_dir_all(output.parent().unwrap()).expect("cannot create the asset directory");
    std::fs::write(&output, &packed).expect("cannot write the compressed asset");
    println!(
        "{} frames: {} -> {} bytes ({:.1}%) at {}",
        raw.len() / FRAME_BYTES,
        raw.len(),
        packed.len(),
        packed.len() as f64 / raw.len() as f64 * 100.0,
        output.display()
    );
}
