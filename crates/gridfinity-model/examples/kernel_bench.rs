use gridfinity_model::{
    Params,
    kernel::{self, Kernel, Legacy, Occt},
};

fn report<K: Kernel>(params: &Params, repetitions: u32) -> Result<(), String> {
    let result = kernel::benchmark::<K>(params, repetitions)?;
    println!(
        "{:<12} build {:>10.3?} total / {:>10.3?} mean | tess {:>10.3?} | {:>8} vertices {:>8} triangles",
        result.kernel,
        result.build_time,
        result.build_time / result.builds,
        result.tessellation_time,
        result.mesh.vertices,
        result.mesh.triangles,
    );
    Ok(())
}

fn main() -> Result<(), String> {
    let repetitions = std::env::args().nth(1).map_or(Ok(3), |value| {
        value
            .parse::<u32>()
            .map_err(|_| format!("invalid repetition count: {value}"))
    })?;
    let params = Params::default();
    report::<Legacy>(&params, repetitions)?;
    report::<Occt>(&params, repetitions)?;
    Ok(())
}
