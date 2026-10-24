//! Switchs de la limitation de fumée des EDC16 (U1, U31, U34).
//!
//! L'EDC15 porte un switch qui dit quel capteur alimente la limitation de
//! fumée ; l'EDC16 a le sien, et il n'est pas seul. Les définitions WinOLS
//! de la famille rangent cinq switchs d'un octet à la suite, dans cet ordre
//! (identifiants Bosch entre parenthèses) :
//!
//! ```text
//!   [T] [BP] 00 02 00 [L] [S] [MM] 0a dd 00
//!    -7  -6           -2  -1   0
//!   T  : température pour la limitation dépendant de la pédale (FlMng_swtAirTemp_C)
//!   BP : pression de suralimentation (0) / pression atmo (1)     (FlMng_swtBPAP_C)
//!   L  : calcul lambda (0) / cartes de fumée MAF ou MAP (1)     (FlMng_swtqLimSmkQ_C)
//!   S  : carte de fumée normale (0) / espace 4D (1)             (FlMng_swtSmokeSpace_C)
//!   MM : MAF (0) / MAP (1)                                      (FlMng_swtSmokeVariant_C)
//! ```
//!
//! Deux d'entre eux sont exposés : MAF/MAP, qui dit quelle carte de fumée
//! est lue, et lambda/cartes, qui dit si ces cartes sont lues du tout. Sur
//! tous les U34 du banc les deux voisins BP et L valent 1 ; sur la plupart
//! des U1 et des U31 ils valent 0, et le calculateur limite alors la fumée
//! par le calcul lambda, les cartes MAF et MAP restant sans effet. C'est le
//! cas du 038906016K de l'issue #31, que la première version du motif, qui
//! exigeait BP = L = 1, ne trouvait pas.
//!
//! Le motif accepte donc 0 ou 1 sur BP et L, avec le reste inchangé :
//! `[0|1] 00 02 00 [0|1] [0|1] [0|1] 0A DD 00`. S accepte aussi 1 : une
//! préparation qui coupe la sonde lambda le passe à 1 (immo-ON-BAK sur
//! 03G906016AB 376708), et les deux switchs n'étaient plus trouvés. Relevé sur le banc EDC16 :
//! 94 des 116 U1, 7 des 17 U31, 196 des 196 U34, jamais plus de deux
//! occurrences par fichier (une par bloc de calibration), et l'adresse
//! trouvée est celle des définitions WinOLS sur les dix packs U1 qui la
//! donnent. Les logiciels restants rangent ces octets autrement, le switch
//! lambda pouvant être à 0x800 octets de là ; on ne détecte que cette
//! disposition plutôt que de nommer un octet au hasard. Demandé par
//! georgiminchev04 sur un 03G906021KH (issue #24), puis par TechPro120 sur
//! le 016K (issue #31).

use std::collections::HashSet;

use crate::models::{DataType, DetectedMap, MapDimensions};

/// Longueur du motif, switch MAF/MAP compris.
const PATTERN_LEN: usize = 10;
/// Position du switch lambda/cartes dans le motif.
const LAMBDA_OFFSET: usize = 4;
/// Position du switch carte de fumée normale / espace 4D dans le motif.
const SMOKE_SPACE_OFFSET: usize = 5;
/// Position du switch MAF/MAP dans le motif.
const MAP_MAF_OFFSET: usize = 6;

/// Ajoute les switchs MAF/MAP et lambda/cartes pour chaque exemplaire trouvé
/// (un par bloc de calibration sur les fichiers qui en ont deux).
/// `with_smoke_space` ajoute le switch S (EDC16U34, réf. XDF 03G906016AB).
pub fn detect_smoke_switches(
    data: &[u8],
    maps: &mut Vec<DetectedMap>,
    detected_addresses: &mut HashSet<u32>,
    with_smoke_space: bool,
) {
    if data.len() < PATTERN_LEN {
        return;
    }

    for i in 0..=data.len() - PATTERN_LEN {
        // BP, un zéro, la constante 02 00, L, S, MM, puis la queue
        // 0A DD 00 : dix octets dont quatre sont des switchs à 0 ou 1.
        if data[i] > 1
            || data[i + 1] != 0x00
            || data[i + 2] != 0x02
            || data[i + 3] != 0x00
            || data[i + LAMBDA_OFFSET] > 1
            || data[i + SMOKE_SPACE_OFFSET] > 1
            || data[i + MAP_MAF_OFFSET] > 1
            || data[i + 7] != 0x0A
            || data[i + 8] != 0xDD
            || data[i + 9] != 0x00
        {
            continue;
        }

        push_switch(
            data,
            maps,
            detected_addresses,
            i + LAMBDA_OFFSET,
            "Lambda/smoke maps switch",
            "Smoke limitation source: 0 = lambda calculation, 1 = MAF or MAP smoke maps",
        );
        push_switch(
            data,
            maps,
            detected_addresses,
            i + MAP_MAF_OFFSET,
            "MAP/MAF switch",
            "Sensor used by the smoke limitation: 0 = MAF, 1 = MAP",
        );
        if with_smoke_space {
            push_switch(
                data,
                maps,
                detected_addresses,
                i + SMOKE_SPACE_OFFSET,
                "Smoke space map switch",
                "Smoke limitation map: 0 = normal smoke map, 1 = 4-D smoke space",
            );
        }
    }
}

fn push_switch(
    data: &[u8],
    maps: &mut Vec<DetectedMap>,
    detected_addresses: &mut HashSet<u32>,
    at: usize,
    name: &str,
    description: &str,
) {
    let addr = at as u32;
    if detected_addresses.contains(&addr) || maps.iter().any(|m| m.address == addr) {
        return;
    }

    let mut map = DetectedMap::new(
        addr,
        1,
        MapDimensions::TwoDimensional { rows: 1, cols: 1 },
        DataType::UInt8,
    );
    map.name = Some(name.to_string());
    // Même dossier que les cartes de fumée des EDC16 (« Smoke Limiter »),
    // pas la catégorie EDC15 « Smoke limitation » qui ouvrait un second dossier.
    map.category = Some("Smoke Limiter".to_string());
    map.description = Some(description.to_string());
    map.unit = Some("-".to_string());
    map.correction_factor = Some(1.0);
    map.offset = Some(0.0);
    map.confidence = 0.95;

    log::debug!("🎯 [EDC16] {} at 0x{:X} = {}", name, addr, data[at]);

    detected_addresses.insert(addr);
    maps.push(map);
}

#[cfg(test)]
mod tests {
    use super::*;

    fn file_with(pattern: &[u8], at: usize) -> Vec<u8> {
        let mut data = vec![0x55u8; at + pattern.len() + 64];
        data[at..at + pattern.len()].copy_from_slice(pattern);
        data
    }

    fn run(data: &[u8]) -> Vec<DetectedMap> {
        let mut maps = Vec::new();
        let mut seen = HashSet::new();
        detect_smoke_switches(data, &mut maps, &mut seen, false);
        maps
    }

    fn named(maps: &[DetectedMap], name: &str) -> Option<u32> {
        maps.iter()
            .find(|m| m.name.as_deref() == Some(name))
            .map(|m| m.address)
    }

    #[test]
    fn finds_both_switches_on_the_u34_layout() {
        // BP = 1, L = 1, MM = 1 : la disposition de tous les U34 du banc.
        let maps = run(&file_with(
            &[0x01, 0x00, 0x02, 0x00, 0x01, 0x00, 0x01, 0x0A, 0xDD, 0x00],
            0x400,
        ));
        assert_eq!(maps.len(), 2);
        assert_eq!(named(&maps, "Lambda/smoke maps switch"), Some(0x404));
        assert_eq!(named(&maps, "MAP/MAF switch"), Some(0x406));
        assert_eq!(maps[0].size, 1);
        // dans le dossier des cartes de fumée EDC16
        assert!(maps.iter().all(|m| m.category.as_deref() == Some("Smoke Limiter")));
    }

    #[test]
    fn finds_the_switches_when_the_ecu_runs_on_lambda() {
        // BP = 0, L = 0, MM = 0 : le 038906016K de l'issue #31.
        let maps = run(&file_with(
            &[0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x0A, 0xDD, 0x00],
            0x100,
        ));
        assert_eq!(maps.len(), 2);
        assert_eq!(named(&maps, "Lambda/smoke maps switch"), Some(0x104));
        assert_eq!(named(&maps, "MAP/MAF switch"), Some(0x106));
    }

    #[test]
    fn finds_the_switches_with_the_smoke_space_set() {
        // S = 1 : préparation sonde lambda coupée (immo-ON-BAK, 03G906016AB)
        let data = file_with(
            &[0x01, 0x00, 0x02, 0x00, 0x01, 0x01, 0x01, 0x0A, 0xDD, 0x00],
            0x300,
        );
        let maps = run(&data);
        assert_eq!(maps.len(), 2);
        assert_eq!(named(&maps, "MAP/MAF switch"), Some(0x306));

        let mut maps = Vec::new();
        let mut seen = HashSet::new();
        detect_smoke_switches(&data, &mut maps, &mut seen, true);
        assert_eq!(maps.len(), 3);
        assert_eq!(named(&maps, "Smoke space map switch"), Some(0x305));
    }

    #[test]
    fn ignores_a_broken_tail() {
        let maps = run(&file_with(
            &[0x01, 0x00, 0x02, 0x00, 0x01, 0x00, 0x01, 0x0A, 0xDE, 0x00],
            0x200,
        ));
        assert!(maps.is_empty());
    }

    #[test]
    fn ignores_a_value_that_is_not_a_switch() {
        // MM = 7, puis L = 2 : aucun des deux n'est un switch.
        let maps = run(&file_with(
            &[0x01, 0x00, 0x02, 0x00, 0x01, 0x00, 0x07, 0x0A, 0xDD, 0x00],
            0x200,
        ));
        assert!(maps.is_empty());
        let maps = run(&file_with(
            &[0x01, 0x00, 0x02, 0x00, 0x02, 0x00, 0x01, 0x0A, 0xDD, 0x00],
            0x200,
        ));
        assert!(maps.is_empty());
    }

    #[test]
    fn ignores_a_changed_constant() {
        // Le 02 00 entre BP et L fait partie du motif.
        let maps = run(&file_with(
            &[0x01, 0x00, 0x03, 0x00, 0x01, 0x00, 0x01, 0x0A, 0xDD, 0x00],
            0x200,
        ));
        assert!(maps.is_empty());
    }

    #[test]
    fn skips_an_address_already_taken() {
        let data = file_with(
            &[0x01, 0x00, 0x02, 0x00, 0x01, 0x00, 0x01, 0x0A, 0xDD, 0x00],
            0x300,
        );
        let mut maps = Vec::new();
        let mut seen = HashSet::from([0x306u32]);
        detect_smoke_switches(&data, &mut maps, &mut seen, false);
        assert_eq!(maps.len(), 1);
        assert_eq!(named(&maps, "Lambda/smoke maps switch"), Some(0x304));
    }
}
