// Copyright Epic Games, Inc. All Rights Reserved.


#include "dna/StreamReader.h"

namespace dna
{
	class FReader : public StreamReader
	{
	public:

		Reader* Geometry;
		Reader* Behaviour;

		// geometry
		virtual std::uint32_t getVertexPositionCount(std::uint16_t meshIndex) const override { return Geometry->getVertexPositionCount(meshIndex); }
		virtual Position getVertexPosition(std::uint16_t meshIndex, std::uint32_t vertexIndex) const override { return Geometry->getVertexPosition(meshIndex, vertexIndex); }
		virtual ConstArrayView<float> getVertexPositionXs(std::uint16_t meshIndex) const override { return Geometry->getVertexPositionXs(meshIndex); }
		virtual ConstArrayView<float> getVertexPositionYs(std::uint16_t meshIndex) const override { return Geometry->getVertexPositionYs(meshIndex); }
		virtual ConstArrayView<float> getVertexPositionZs(std::uint16_t meshIndex) const override { return Geometry->getVertexPositionZs(meshIndex); }
		virtual std::uint32_t getVertexTextureCoordinateCount(std::uint16_t meshIndex) const override { return Geometry->getVertexTextureCoordinateCount(meshIndex); }
		virtual TextureCoordinate getVertexTextureCoordinate(std::uint16_t meshIndex, std::uint32_t textureCoordinateIndex) const override { return Geometry->getVertexTextureCoordinate(meshIndex, textureCoordinateIndex); }
		virtual ConstArrayView<float> getVertexTextureCoordinateUs(std::uint16_t meshIndex) const override { return Geometry->getVertexTextureCoordinateUs(meshIndex); }
		virtual ConstArrayView<float> getVertexTextureCoordinateVs(std::uint16_t meshIndex) const override { return Geometry->getVertexTextureCoordinateVs(meshIndex); }
		virtual std::uint32_t getVertexNormalCount(std::uint16_t meshIndex) const override { return Geometry->getVertexNormalCount(meshIndex); }
		virtual Normal getVertexNormal(std::uint16_t meshIndex, std::uint32_t normalIndex) const override { return Geometry->getVertexNormal(meshIndex, normalIndex); }
		virtual ConstArrayView<float> getVertexNormalXs(std::uint16_t meshIndex) const override { return Geometry->getVertexNormalXs(meshIndex); }
		virtual ConstArrayView<float> getVertexNormalYs(std::uint16_t meshIndex) const override { return Geometry->getVertexNormalYs(meshIndex); }
		virtual ConstArrayView<float> getVertexNormalZs(std::uint16_t meshIndex) const override { return Geometry->getVertexNormalZs(meshIndex); }
		virtual std::uint32_t getVertexLayoutCount(std::uint16_t meshIndex) const override { return Geometry->getVertexLayoutCount(meshIndex); }
		virtual VertexLayout getVertexLayout(std::uint16_t meshIndex, std::uint32_t layoutIndex) const override { return Geometry->getVertexLayout(meshIndex, layoutIndex); }
		virtual ConstArrayView<std::uint32_t> getVertexLayoutPositionIndices(std::uint16_t meshIndex) const override { return Geometry->getVertexLayoutPositionIndices(meshIndex); }
		virtual ConstArrayView<std::uint32_t> getVertexLayoutTextureCoordinateIndices(std::uint16_t meshIndex) const override { return Geometry->getVertexLayoutTextureCoordinateIndices(meshIndex); }
		virtual ConstArrayView<std::uint32_t> getVertexLayoutNormalIndices(std::uint16_t meshIndex) const override { return Geometry->getVertexLayoutNormalIndices(meshIndex); }
		virtual std::uint32_t getFaceCount(std::uint16_t meshIndex) const override { return Geometry->getFaceCount(meshIndex); }
		virtual ConstArrayView<std::uint32_t> getFaceVertexLayoutIndices(std::uint16_t meshIndex, std::uint32_t faceIndex) const override { return Geometry->getFaceVertexLayoutIndices(meshIndex, faceIndex); }
		virtual std::uint16_t getMaximumInfluencePerVertex(std::uint16_t meshIndex) const override { return Geometry->getMaximumInfluencePerVertex(meshIndex); }
		virtual std::uint32_t getSkinWeightsCount(std::uint16_t meshIndex) const override { return Geometry->getSkinWeightsCount(meshIndex); }
		virtual ConstArrayView<float> getSkinWeightsValues(std::uint16_t meshIndex, std::uint32_t vertexIndex) const override { return Geometry->getSkinWeightsValues(meshIndex, vertexIndex); }
		virtual ConstArrayView<std::uint16_t> getSkinWeightsJointIndices(std::uint16_t meshIndex, std::uint32_t vertexIndex) const override { return Geometry->getSkinWeightsJointIndices(meshIndex, vertexIndex); }
		virtual std::uint16_t getBlendShapeTargetCount(std::uint16_t meshIndex) const override { return Geometry->getBlendShapeTargetCount(meshIndex); }
		virtual std::uint16_t getBlendShapeChannelIndex(std::uint16_t meshIndex, std::uint16_t blendShapeTargetIndex) const override { return Geometry->getBlendShapeChannelIndex(meshIndex, blendShapeTargetIndex); }
		virtual std::uint32_t getBlendShapeTargetDeltaCount(std::uint16_t meshIndex, std::uint16_t blendShapeTargetIndex) const override { return Geometry->getBlendShapeTargetDeltaCount(meshIndex, blendShapeTargetIndex); }
		virtual Delta getBlendShapeTargetDelta(std::uint16_t meshIndex, std::uint16_t blendShapeTargetIndex, std::uint32_t deltaIndex) const override { return Geometry->getBlendShapeTargetDelta(meshIndex, blendShapeTargetIndex, deltaIndex); }
		virtual ConstArrayView<float> getBlendShapeTargetDeltaXs(std::uint16_t meshIndex, std::uint16_t blendShapeTargetIndex) const override { return Geometry->getBlendShapeTargetDeltaXs(meshIndex, blendShapeTargetIndex); }
		virtual ConstArrayView<float> getBlendShapeTargetDeltaYs(std::uint16_t meshIndex, std::uint16_t blendShapeTargetIndex) const override { return Geometry->getBlendShapeTargetDeltaYs(meshIndex, blendShapeTargetIndex); }
		virtual ConstArrayView<float> getBlendShapeTargetDeltaZs(std::uint16_t meshIndex, std::uint16_t blendShapeTargetIndex) const override { return Geometry->getBlendShapeTargetDeltaZs(meshIndex, blendShapeTargetIndex); }
		virtual ConstArrayView<std::uint32_t> getBlendShapeTargetVertexIndices(std::uint16_t meshIndex, std::uint16_t blendShapeTargetIndex) const override { return Geometry->getBlendShapeTargetVertexIndices(meshIndex, blendShapeTargetIndex); }

		// behaviour
		virtual ConstArrayView<std::uint16_t> getGUIToRawInputIndices() const override { return Behaviour->getGUIToRawInputIndices(); }
		virtual ConstArrayView<std::uint16_t> getGUIToRawOutputIndices() const override { return Behaviour->getGUIToRawOutputIndices(); }
		virtual ConstArrayView<float> getGUIToRawFromValues() const override { return Behaviour->getGUIToRawFromValues(); }
		virtual ConstArrayView<float> getGUIToRawToValues() const override { return Behaviour->getGUIToRawToValues(); }
		virtual ConstArrayView<float> getGUIToRawSlopeValues() const override { return Behaviour->getGUIToRawSlopeValues(); }
		virtual ConstArrayView<float> getGUIToRawCutValues() const override { return Behaviour->getGUIToRawCutValues(); }
		virtual std::uint16_t getPSDCount() const override { return Behaviour->getPSDCount(); }
		virtual ConstArrayView<std::uint16_t> getPSDRowIndices() const override { return Behaviour->getPSDRowIndices(); }
		virtual ConstArrayView<std::uint16_t> getPSDColumnIndices() const override { return Behaviour->getPSDColumnIndices(); }
		virtual ConstArrayView<float> getPSDValues() const override { return Behaviour->getPSDValues(); }
		virtual std::uint16_t getJointRowCount() const override { return Behaviour->getJointRowCount(); }
		virtual std::uint16_t getJointColumnCount() const override { return Behaviour->getJointColumnCount(); }
		virtual ConstArrayView<std::uint16_t> getJointVariableAttributeIndices(std::uint16_t lod) const override { return Behaviour->getJointVariableAttributeIndices(lod); }
		virtual std::uint16_t getJointGroupCount() const override { return Behaviour->getJointGroupCount(); }
		virtual ConstArrayView<std::uint16_t> getJointGroupLODs(std::uint16_t jointGroupIndex) const override { return Behaviour->getJointGroupLODs(jointGroupIndex); }
		virtual ConstArrayView<std::uint16_t> getJointGroupInputIndices(std::uint16_t jointGroupIndex) const override { return Behaviour->getJointGroupInputIndices(jointGroupIndex); }
		virtual ConstArrayView<std::uint16_t> getJointGroupOutputIndices(std::uint16_t jointGroupIndex) const override { return Behaviour->getJointGroupOutputIndices(jointGroupIndex); }
		virtual ConstArrayView<float> getJointGroupValues(std::uint16_t jointGroupIndex) const override { return Behaviour->getJointGroupValues(jointGroupIndex); }
		virtual ConstArrayView<std::uint16_t> getJointGroupJointIndices(std::uint16_t jointGroupIndex) const override { return Behaviour->getJointGroupJointIndices(jointGroupIndex); }
		virtual ConstArrayView<std::uint16_t> getBlendShapeChannelLODs() const override { return Behaviour->getBlendShapeChannelLODs(); }
		virtual ConstArrayView<std::uint16_t> getBlendShapeChannelInputIndices() const override { return Behaviour->getBlendShapeChannelInputIndices(); }
		virtual ConstArrayView<std::uint16_t> getBlendShapeChannelOutputIndices() const override { return Behaviour->getBlendShapeChannelOutputIndices(); }
		virtual ConstArrayView<std::uint16_t> getAnimatedMapLODs() const override { return Behaviour->getAnimatedMapLODs(); }
		virtual ConstArrayView<std::uint16_t> getAnimatedMapInputIndices() const override { return Behaviour->getAnimatedMapInputIndices(); }
		virtual ConstArrayView<std::uint16_t> getAnimatedMapOutputIndices() const override { return Behaviour->getAnimatedMapOutputIndices(); }
		virtual ConstArrayView<float> getAnimatedMapFromValues() const override { return Behaviour->getAnimatedMapFromValues(); }
		virtual ConstArrayView<float> getAnimatedMapToValues() const override { return Behaviour->getAnimatedMapToValues(); }
		virtual ConstArrayView<float> getAnimatedMapSlopeValues() const override { return Behaviour->getAnimatedMapSlopeValues(); }
		virtual ConstArrayView<float> getAnimatedMapCutValues() const override { return Behaviour->getAnimatedMapCutValues(); }

		// definition
		virtual std::uint16_t getGUIControlCount() const override { return Behaviour->getGUIControlCount(); }
		virtual StringView getGUIControlName(std::uint16_t index) const override { return Behaviour->getGUIControlName(index); }
		virtual std::uint16_t getRawControlCount() const override { return Behaviour->getRawControlCount(); }
		virtual StringView getRawControlName(std::uint16_t index) const override { return Behaviour->getRawControlName(index); }
		virtual std::uint16_t getJointCount() const override { return Behaviour->getJointCount(); }
		virtual StringView getJointName(std::uint16_t index) const override { return Behaviour->getJointName(index); }
		virtual std::uint16_t getJointIndexListCount() const override { return Behaviour->getJointIndexListCount(); }
		virtual ConstArrayView<std::uint16_t> getJointIndicesForLOD(std::uint16_t lod) const override { return Behaviour->getJointIndicesForLOD(lod); }
		virtual std::uint16_t getJointParentIndex(std::uint16_t index) const override { return Behaviour->getJointParentIndex(index); }
		virtual std::uint16_t getBlendShapeChannelCount() const override { return Behaviour->getBlendShapeChannelCount(); }
		virtual StringView getBlendShapeChannelName(std::uint16_t index) const override { return Behaviour->getBlendShapeChannelName(index); }
		virtual std::uint16_t getBlendShapeChannelIndexListCount() const override { return Behaviour->getBlendShapeChannelIndexListCount(); }
		virtual ConstArrayView<std::uint16_t> getBlendShapeChannelIndicesForLOD(std::uint16_t lod) const override { return Behaviour->getBlendShapeChannelIndicesForLOD(lod); }
		virtual std::uint16_t getAnimatedMapCount() const override { return Behaviour->getAnimatedMapCount(); }
		virtual StringView getAnimatedMapName(std::uint16_t index) const override { return Behaviour->getAnimatedMapName(index); }
		virtual std::uint16_t getAnimatedMapIndexListCount() const override { return Behaviour->getAnimatedMapIndexListCount(); }
		virtual ConstArrayView<std::uint16_t> getAnimatedMapIndicesForLOD(std::uint16_t lod) const override { return Behaviour->getAnimatedMapIndicesForLOD(lod); }
		virtual std::uint16_t getMeshCount() const override { return Behaviour->getMeshCount(); }
		virtual StringView getMeshName(std::uint16_t index) const override { return Behaviour->getMeshName(index); }
		virtual std::uint16_t getMeshIndexListCount() const override { return Behaviour->getMeshIndexListCount(); }
		virtual ConstArrayView<std::uint16_t> getMeshIndicesForLOD(std::uint16_t lod) const override { return Behaviour->getMeshIndicesForLOD(lod); }
		virtual std::uint16_t getMeshBlendShapeChannelMappingCount() const override { return Behaviour->getMeshBlendShapeChannelMappingCount(); }
		virtual MeshBlendShapeChannelMapping getMeshBlendShapeChannelMapping(std::uint16_t index) const override { return Behaviour->getMeshBlendShapeChannelMapping(index); }
		virtual ConstArrayView<std::uint16_t> getMeshBlendShapeChannelMappingIndicesForLOD(std::uint16_t lod) const override { return Behaviour->getMeshBlendShapeChannelMappingIndicesForLOD(lod); }
		virtual Vector3 getNeutralJointTranslation(std::uint16_t index) const override { return Behaviour->getNeutralJointTranslation(index); }
		virtual ConstArrayView<float> getNeutralJointTranslationXs() const override { return Behaviour->getNeutralJointTranslationXs(); }
		virtual ConstArrayView<float> getNeutralJointTranslationYs() const override { return Behaviour->getNeutralJointTranslationYs(); }
		virtual ConstArrayView<float> getNeutralJointTranslationZs() const override { return Behaviour->getNeutralJointTranslationZs(); }
		virtual Vector3 getNeutralJointRotation(std::uint16_t index) const override { return Behaviour->getNeutralJointRotation(index); }
		virtual ConstArrayView<float> getNeutralJointRotationXs() const override { return Behaviour->getNeutralJointRotationXs(); }
		virtual ConstArrayView<float> getNeutralJointRotationYs() const override { return Behaviour->getNeutralJointRotationYs(); }
		virtual ConstArrayView<float> getNeutralJointRotationZs() const override { return Behaviour->getNeutralJointRotationZs(); }

		// description
		virtual StringView getName() const override { return Behaviour->getName(); }
		virtual Archetype getArchetype() const override { return Behaviour->getArchetype(); }
		virtual Gender getGender() const override { return Behaviour->getGender(); }
		virtual std::uint16_t getAge() const override { return Behaviour->getAge(); }
		virtual std::uint32_t getMetaDataCount() const override { return Behaviour->getMetaDataCount(); }
		virtual StringView getMetaDataKey(std::uint32_t index) const override { return Behaviour->getMetaDataKey(index); }
		virtual StringView getMetaDataValue(const char* key) const override { return Behaviour->getMetaDataValue(key); }
		virtual TranslationUnit getTranslationUnit() const override { return Behaviour->getTranslationUnit(); }
		virtual RotationUnit getRotationUnit() const override { return Behaviour->getRotationUnit(); }
		virtual CoordinateSystem getCoordinateSystem() const override { return Behaviour->getCoordinateSystem(); }
		virtual std::uint16_t getLODCount() const override { return Behaviour->getLODCount(); }
		virtual std::uint16_t getDBMaxLOD() const override { return Behaviour->getDBMaxLOD(); }
		virtual StringView getDBComplexity() const override { return Behaviour->getDBComplexity(); }
		virtual StringView getDBName() const override { return Behaviour->getDBName(); }

		// reader
		virtual void unload(dna::DataLayer Layer) override { ensureMsgf(false, TEXT("Assest are not unloadable")); }

		// stream reader

		virtual void read() override { }
	};
}
