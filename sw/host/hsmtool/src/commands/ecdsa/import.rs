// Copyright lowRISC contributors (OpenTitan project).
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

use anyhow::Result;
use cryptoki::session::Session;
use serde::{Deserialize, Serialize};
use std::any::Any;
use std::path::PathBuf;
use std::str::FromStr;

use crate::commands::{BasicResult, Dispatch};
use crate::error::HsmError;
use crate::module::Module;
use crate::util::attribute::{AttrData, AttributeMap, AttributeType};
use crate::util::helper;
use crate::util::key::ecdsa::{load_private_key, load_public_key};
use crate::util::wrap::Wrap;

#[derive(clap::Args, Debug, Serialize, Deserialize)]
pub struct Import {
    #[arg(long)]
    id: Option<String>,
    #[arg(short, long)]
    label: Option<String>,
    /// The key is a public key only.
    #[arg(short, long)]
    public: bool,
    /// Attributes to apply to the public key.
    #[arg(long)]
    public_attrs: Option<AttributeMap>,
    /// Attributes to apply to the private key.
    #[arg(long)]
    private_attrs: Option<AttributeMap>,
    /// Unwrap the imported key with a wrapping key.
    #[arg(long)]
    unwrap: Option<String>,
    filename: PathBuf,
}

impl Import {
    const PUBLIC_ATTRS: &str = r#"{
        "CKA_TOKEN": true,
        "CKA_CLASS": "CKO_PUBLIC_KEY",
        "CKA_KEY_TYPE": "CKK_EC",
        "CKA_VERIFY": true
    }"#;

    const PRIVATE_ATTRS: &str = r#"{
        "CKA_TOKEN": true,
        "CKA_PRIVATE": true,
        "CKA_SENSITIVE": true,
        "CKA_CLASS": "CKO_PRIVATE_KEY",
        "CKA_KEY_TYPE": "CKK_EC",
        "CKA_SIGN": true
    }"#;

    fn unwrap_key(&self, session: &Session, template: &AttributeMap) -> Result<()> {
        let key = helper::read_file(&self.filename)?;
        let wrapper = Wrap::AesKeyWrapPad;
        let _key = wrapper.unwrap(session, key.as_slice(), self.unwrap.as_deref(), template)?;
        Ok(())
    }
}

#[typetag::serde(name = "ecdsa-import")]
impl Dispatch for Import {
    fn run(
        &self,
        _context: &dyn Any,
        _hsm: &Module,
        session: Option<&Session>,
    ) -> Result<Box<dyn erased_serde::Serialize>> {
        let session = session.ok_or(HsmError::SessionRequired)?;
        helper::no_object_exists(session, self.id.as_deref(), self.label.as_deref())?;
        let mut public_attrs =
            AttributeMap::from_str(Self::PUBLIC_ATTRS).expect("error in PUBLIC_ATTRS");
        let mut private_attrs =
            AttributeMap::from_str(Self::PRIVATE_ATTRS).expect("error in PRIVATE_ATTRS");

        let id = AttrData::Str(self.id.as_ref().cloned().unwrap_or_else(helper::random_id));
        let result = Box::new(BasicResult {
            success: true,
            id: id.clone(),
            label: AttrData::Str(self.label.as_ref().cloned().unwrap_or_default()),
            value: None,
            error: None,
        });
        public_attrs.insert(AttributeType::Id, id.clone());
        public_attrs.insert(AttributeType::Label, result.label.clone());
        if let Some(tpl) = &self.public_attrs {
            public_attrs.merge(tpl.clone());
        }

        private_attrs.insert(AttributeType::Id, id);
        private_attrs.insert(AttributeType::Label, result.label.clone());
        if let Some(tpl) = &self.private_attrs {
            private_attrs.merge(tpl.clone());
        }

        if self.public {
            let key = load_public_key(&self.filename)?;
            public_attrs.merge(AttributeMap::try_from(&key)?);
            let _pubkey = session.create_object(&public_attrs.to_vec()?)?;
        } else if self.unwrap.is_some() {
            self.unwrap_key(session, &private_attrs)?;
        } else {
            let key = load_private_key(&self.filename)?;
            public_attrs.merge(AttributeMap::try_from(key.verifying_key())?);
            let _pubkey = session.create_object(&public_attrs.to_vec()?)?;
            private_attrs.merge(AttributeMap::try_from(&key)?);
            let _privkey = session.create_object(&private_attrs.to_vec()?)?;
        }
        Ok(result)
    }
}
